set(LIBNAME mavsdk_rc_passthrough)

add_library(${LIBNAME}
    PluginRC.cpp
    PluginRC_impl.cpp
)

target_link_libraries(${LIBNAME}
    mavsdk
    mavsdk_mavlink_passthrough
)

set_target_properties(${LIBNAME}
    PROPERTIES COMPILE_FLAGS ${warnings}
    VERSION ${MAVSDK_VERSION_STRING}
    SOVERSION ${MAVSDK_SOVERSION_STRING}
)

target_include_directories(${LIBNAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/mavsdk>
    )

install(TARGETS ${LIBNAME}
    EXPORT mavsdk-targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    )

install(FILES
    include/plugins/rc_passthrough/PluginRC.h
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/mavsdk/plugins/rc_passthrough
)

