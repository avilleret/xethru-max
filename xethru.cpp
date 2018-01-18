/**
	@file
	xethru - access Xethru X4M200 breath sensor

	@ingroup sensor
*/

#include "ext.h"
#include "ext_obex.h"
#include "jpatcher_api.h"
#include "ext_systhread.h"

#include <string>
#include <fstream>

#include "ModuleConnector.hpp"
#include "DataRecorder.hpp"
#include "X4M200.hpp"
#include "xtid.h"

enum xethru_sensor {
    XETHRU_UNKNOWN = 0,
    XETHRU_X4M200
};

struct t_xethru {
	t_object	ob;
    void* sleep_out;
    bool thread_cancel;
    t_systhread thread_sleep;
    t_systhread thread_resp;
    t_systhread_mutex	mutex_sleep;							// mutual exclusion lock for threadsafety
    t_systhread_mutex	mutex_resp;							// mutual exclusion lock for threadsafety

    XeThru::SleepData sleep_data;
    XeThru::RespirationData resp_data;

    xethru_sensor sensor_type;
    std::string interface;
    XeThru::ModuleConnector* module_connector;
    XeThru::DataRecorder* recorder;
};

void xethru_connect(t_xethru* x, t_symbol* s);

static t_class* xethru_class = nullptr;

void* xethru_new(t_symbol* name, long argc, t_atom* argv) {
    
    auto x = (t_xethru*) object_alloc(xethru_class);
    
    x->sleep_out = outlet_new(x, 0);

    x->thread_cancel = false;

    x->thread_sleep = NULL;
    x->thread_resp = NULL;

    systhread_mutex_new(&x->mutex_sleep,0);
    systhread_mutex_new(&x->mutex_resp,0);

    if(argc > 0 && argv[0].a_type == A_SYM)
    {
        if ( argv[0].a_w.w_sym == gensym("X4M200") )
            x->sensor_type = xethru_sensor::XETHRU_X4M200;
    }
    
    if(argc > 1 && argv[1].a_type == A_SYM)
    {
        xethru_connect(x, argv[1].a_w.w_sym);
    }
    
    return x;
}

void xethru_bang(t_xethru* x)
{
    if(!x->module_connector)
        return;


    t_atom atom_sleep[7];

    systhread_mutex_lock(x->mutex_sleep);
    {
        atom_setlong(atom_sleep,    x->sleep_data.frame_counter);
        atom_setlong(atom_sleep+1,  x->sleep_data.sensor_state);
        atom_setfloat(atom_sleep+2, x->sleep_data.respiration_rate);
        atom_setfloat(atom_sleep+3, x->sleep_data.distance);
        atom_setlong(atom_sleep+4,  x->sleep_data.signal_quality);
        atom_setfloat(atom_sleep+5, x->sleep_data.movement_slow);
        atom_setfloat(atom_sleep+6, x->sleep_data.movement_fast);
    }
    systhread_mutex_unlock(x->mutex_sleep);

    outlet_anything(x->sleep_out, gensym("respiration_data"), 7, atom_sleep);

    /*
    t_atom atom_resp[6];
    systhread_mutex_lock(x->mutex_resp);
    {
        atom_setlong(atom_resp,    x->resp_data.frame_counter);
        atom_setlong(atom_resp+1,  x->resp_data.sensor_state);
        atom_setfloat(atom_resp+2, x->resp_data.distance);
        atom_setfloat(atom_resp+3, x->resp_data.respiration_rate);
        atom_setlong(atom_resp+4,  x->resp_data.movement);
        atom_setfloat(atom_resp+5, x->resp_data.signal_quality);
    }
    systhread_mutex_unlock(x->mutex_resp);

    outlet_anything(x->resp_out, gensym("respiration_data"), 7, atom_resp);
    */
}

void *xethru_threadproc_sleep(t_xethru *x)
{
    XeThru::X4M200 &x4m200 = x->module_connector->get_x4m200();
    XeThru::SleepData sleep_data;

    while(!x->thread_cancel)
    {
        int err = x4m200.read_message_respiration_sleep(&sleep_data);
        if (!err)
        {
            systhread_mutex_lock(x->mutex_sleep);
            x->sleep_data = sleep_data;															// fiddle with shared data
            systhread_mutex_unlock(x->mutex_sleep);
        }
    }
}

void *xethru_threadproc_resp(t_xethru *x)
{
    XeThru::X4M200 &x4m200 = x->module_connector->get_x4m200();
    XeThru::RespirationData resp_data;

    while(!x->thread_cancel)
    {
        int err = x4m200.read_message_respiration_legacy(&resp_data);
        if (!err)
        {
            systhread_mutex_lock(x->mutex_resp);
            x->resp_data = resp_data;															// fiddle with shared data
            systhread_mutex_unlock(x->mutex_resp);
        }
    }
}

void xethru_record(t_xethru* x, t_int flag)
{
    const XeThru::DataTypes data_types =
    XeThru::BasebandApDataType | XeThru::SleepDataType; //specifying data_types
 
    if (flag)
    {
        const std::string output_directory = "."; //setting directory to current
        
        //start recorder
        int res = x->recorder->start_recording(data_types, output_directory);
        if (res != 0) {
            //Start recording failed
            object_error((t_object*) x, "can't start recording");
        }
        
        t_atom a;
        atom_setlong(&a, res != 0);
        outlet_anything(x->sleep_out, gensym("record"), 1, &a);
    }
    else
    {
        x->recorder->stop_recording(data_types);
        
        t_atom a;
        atom_setlong(&a, 0);
        outlet_anything(x->sleep_out, gensym("record"), 1, &a);
    }
}

void xethru_configure(t_xethru*x)
{
    //configure and run X4M200
    XeThru::X4M200 &x4m200 = x->module_connector->get_x4m200();
    x4m200.load_profile(XTS_ID_APP_RESPIRATION_2);
    x4m200.set_output_control(XTS_ID_RESPIRATION_DETECTIONLIST, XTID_OUTPUT_CONTROL_ENABLE);
    x4m200.set_sensor_mode(XTID_SM_RUN, 0);
}

void xethru_disconnect(t_xethru* x) {
    if (x->module_connector)
    {

        unsigned int ret;

        x->thread_cancel = true;						// tell the thread to stop

        if (x->thread_sleep) {
            post("stopping our thread");
            systhread_join(x->thread_sleep, &ret);					// wait for the thread to stop
            x->thread_sleep = NULL;
        }

        if (x->thread_resp) {
            post("stopping our thread");
            systhread_join(x->thread_resp, &ret);					// wait for the thread to stop
            x->thread_resp = NULL;
        }

        x->thread_cancel = false;


        delete x->module_connector;
        x->module_connector = nullptr;
        x->recorder = nullptr;
    }
}

void xethru_connect(t_xethru* x, t_symbol* s)
{
    if (x->module_connector)
    {
        xethru_disconnect(x);
    }
    
    x->module_connector = new XeThru::ModuleConnector(s->s_name);
    if (x->module_connector)
    {
      x->interface =  s->s_name;
      x->recorder = &x->module_connector->get_data_recorder();

      // create new thread + begin execution
      if (x->thread_sleep == NULL) {
          systhread_create((method) xethru_threadproc_sleep, x, 0, 0, 0, &x->thread_sleep);
      }
      if (x->thread_resp == NULL) {
          systhread_create((method) xethru_threadproc_resp,  x, 0, 0, 0, &x->thread_resp);
      }
    }
    else
        object_error((t_object*)x, "Can't initialize sensor at '%s'", s->s_name);
}

void xethru_free(t_xethru* x) {
    xethru_disconnect(x);

    // free out mutex
    if (x->mutex_sleep)
        systhread_mutex_free(x->mutex_sleep);

    if (x->mutex_resp)
        systhread_mutex_free(x->mutex_resp);
}

void ext_main(void* r) {
	xethru_class = class_new("xethru", (method)xethru_new, (method)xethru_free, sizeof(t_xethru), 0L, A_GIMME, 0);
	
    class_addmethod(xethru_class, (method)xethru_record, "record", A_LONG, 0);
    class_addmethod(xethru_class, (method)xethru_disconnect, "disconnect", 0);
    class_addmethod(xethru_class, (method)xethru_connect, "connect", A_SYM, 0);
    class_addmethod(xethru_class, (method)xethru_bang, "bang", 0);

	class_register(CLASS_BOX, xethru_class);
}
