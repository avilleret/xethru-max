cmake_minimum_required(VERSION 3.5)
project(xethru)

set(CMAKE_MODULE_PATH
  "${CMAKE_MODULE_PATH}"
  "${CMAKE_CURRENT_SOURCE_DIR}")

# disable i386 build because XeThru provide 64bit only library
set(CMAKE_OSX_ARCHITECTURES x86_64)

set(XETHRU_SDK_ROOT "" CACHE PATH "Root path of Xethru Module Connector")
set(XETHRU_INSTALL_FOLDER "$ENV{HOME}/Documents/Max 7/Packages" CACHE PATH "Package installation folder")

if(APPLE)
    #set(CMAKE_INSTALL_RPATH "@loader_path/../../../../support;@loader_path")
elseif(WIN32)
    set(CMAKE_INSTALL_RPATH  "\$ORIGIN/../support;\$ORIGIN")
endif()


find_package(MaxSDK QUIET)

if(APPLE)
    FIND_PATH(FILE_H_DIR Files.h
        HINTS
        /System/Library/Frameworks/
        /System/Library/Frameworks/CoreServices.framework/Versions/A/Frameworks/CarbonCore.framework/Versions/A/Headers/
        /Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX.sdk/System/Library/Frameworks/CoreServices.framework/Versions/A/Frameworks/CarbonCore.framework/Versions/A/Headers/
        /Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX10.12.sdk/System/Library/Frameworks/CoreServices.framework/Versions/A/Frameworks/CarbonCore.framework/Versions/A/Headers
        /Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX10.11.sdk/System/Library/Frameworks/CoreServices.framework/Versions/A/Frameworks/CarbonCore.framework/Versions/A/Headers
        /Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX10.10.sdk/System/Library/Frameworks/CoreServices.framework/Versions/A/Frameworks/CarbonCore.framework/Versions/A/Headers
        /Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX10.9.sdk/System/Library/Frameworks/CoreServices.framework/Versions/A/Frameworks/CarbonCore.framework/Versions/A/Headers)
    message( STATUS " Files.h have been found here : ${FILE_H_DIR}")
    if( ${FILE_H_DIR} EQUAL "FILE_H_DIR-NOT_FOUND")
        message( STATUS "  Files.h NOT FOUND !!" )
    else()
        include_directories("${FILE_H_DIR}")
    endif()
endif()

include_directories( 
	"${XETHRU_SDK_ROOT}/include"    
	${MAXSDK_MAX_INCLUDE_DIR})

link_directories("${XETHRU_SDK_ROOT}/lib"
)


add_library( 
	${PROJECT_NAME} 
	MODULE
	${PROJECT_NAME}.cpp
)



if(APPLE)
    set_target_properties(${PROJECT_NAME} PROPERTIES BUNDLE True)
    set_target_properties(${PROJECT_NAME} PROPERTIES BUNDLE_EXTENSION "mxo")

    target_link_libraries(
		${PROJECT_NAME}
		PRIVATE
		ModuleConnector
	)
    target_link_libraries(${PROJECT_NAME} PRIVATE -Wl,-undefined,dynamic_lookup)
    file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/PkgInfo DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.mxo/Contents/)
elseif(WIN32)
    if("${CMAKE_SIZEOF_VOID_P}" MATCHES "8")
        set_target_properties(${PROJECT_NAME} PROPERTIES SUFFIX ".mxe64")
    else()
        set_target_properties(${PROJECT_NAME} PROPERTIES SUFFIX ".mxe")
    endif()
endif()

if(APPLE)
	install(
		FILES
		 	${XETHRU_SDK_ROOT}/lib/libModuleConnector.dylib
		DESTINATION
			"${XETHRU_INSTALL_FOLDER}/xethru/support"
		COMPONENT
            ${PROJECT_NAME}
	)
else()
	install(
		FILES
		 ${PROJECT_NAME}
		 	${XETHRU_SDK_ROOT}/lib/libModuleConnector.dll
		DESTINATION
			"${XETHRU_INSTALL_FOLDER}/xethru/support"
		COMPONENT
            ${PROJECT_NAME}
	)  
endif()

install(
 	TARGETS
 		${PROJECT_NAME}
 	BUNDLE DESTINATION
        "${XETHRU_INSTALL_FOLDER}/xethru/externals"
    RUNTIME DESTINATION
        "${XETHRU_INSTALL_FOLDER}/xethru/externals"
    LIBRARY DESTINATION
        "${XETHRU_INSTALL_FOLDER}/xethru/externals"
    COMPONENT
        ${PROJECT_NAME}
 	)
