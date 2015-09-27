
set( BUILD_Epiphan false CACHE BOOL force )

if( BUILD_Epiphan )
    
    message( STATUS "HAL: building 'Epiphan' camera driver.")

    # build V2U library
    include_directories( SDK/includes )

    set( V2U_FILES SDK/v2u_lib/src/v2u_bmp.c
                    SDK/v2u_lib/src/v2u_jpeg.c
                    SDK/v2u_lib/src/v2u_pal.c
                    SDK/v2u_lib/src/v2u_png.c
                    SDK/v2u_lib/src/v2u_sys.c
                    SDK/v2u_lib/src/v2u_util.c )


    # build SLAVA library
    set( SLAVA_FILES SDK/slib/src/s_base32.c
                        SDK/slib/src/s_base64.c
                        SDK/slib/src/s_bitset.c
                        SDK/slib/src/s_buf.c
                        SDK/slib/src/s_cs.c
                        SDK/slib/src/s_dom.c
                        SDK/slib/src/s_event.c
                        SDK/slib/src/s_file.c
                        SDK/slib/src/s_fio.c
                        SDK/slib/src/s_fmem.c
                        SDK/slib/src/s_fnull.c
                        SDK/slib/src/s_fsock.c
                        SDK/slib/src/s_fsplit.c
                        SDK/slib/src/s_fsub.c
                        SDK/slib/src/s_futil.c
                        SDK/slib/src/s_fwrap.c
                        SDK/slib/src/s_fzio.c
                        SDK/slib/src/s_fzip.c
                        SDK/slib/src/s_hash.c
                        SDK/slib/src/s_hist.c
                        SDK/slib/src/s_init.c
                        SDK/slib/src/s_itr.c
                        SDK/slib/src/s_itra.c
                        SDK/slib/src/s_itrc.c
                        SDK/slib/src/s_itrf.c
                        SDK/slib/src/s_itrs.c
                        SDK/slib/src/s_lib.c
                        SDK/slib/src/s_lock.c
                        SDK/slib/src/s_math.c
                        SDK/slib/src/s_md.c
                        SDK/slib/src/s_md5.c
                        SDK/slib/src/s_mem.c
                        SDK/slib/src/s_mpm.c
                        SDK/slib/src/s_mutex.c
                        SDK/slib/src/s_net.c
                        SDK/slib/src/s_opt.c
                        SDK/slib/src/s_parse.c
                        SDK/slib/src/s_prop.c
                        SDK/slib/src/s_propx.c
                        SDK/slib/src/s_ring.c
                        SDK/slib/src/s_rwlock.c
                        SDK/slib/src/s_queue.c
                        SDK/slib/src/s_random.c
                        SDK/slib/src/s_sha1.c
                        SDK/slib/src/s_stack.c
                        SDK/slib/src/s_str.c
                        SDK/slib/src/s_strbuf.c
                        SDK/slib/src/s_thread.c
                        SDK/slib/src/s_time.c
                        SDK/slib/src/s_trace.c
                        SDK/slib/src/s_utf8.c
                        SDK/slib/src/s_util.c
                        SDK/slib/src/s_vector.c
                        SDK/slib/src/s_wkq.c
                        SDK/slib/src/s_xml.c
                        SDK/slib/src/unix/u_event.c
                        SDK/slib/src/unix/u_furl.c
                        SDK/slib/src/unix/u_futil.c
                        SDK/slib/src/unix/u_mutex.c
                        SDK/slib/src/unix/u_thread.c
                        SDK/slib/src/unix/u_trace.c )

    # ignore warnings generated from third party includes
    if(CMAKE_COMPILER_IS_GNUCXX)
        set_source_files_properties( ${SLAVA_FILES} ${V2U_FILES} PROPERTIES COMPILE_FLAGS " -w " )
    endif()

#    add_library( v2u STATIC ${V2U_FILES} )
#    add_library( slava STATIC ${SLAVA_FILES} )


    # copy proprietary library frmgrab
    if(APPLE)
        file( COPY SDK/libfrmgrab/macosx/libfrmgrab.a DESTINATION ${CMAKE_CURRENT_BINARY_DIR} )
    else()
        file( COPY SDK/libfrmgrab/linux/x86_64/libfrmgrab.a DESTINATION ${CMAKE_CURRENT_BINARY_DIR} )
    endif()

    add_custom_target( frmgrab )
    set_property( TARGET frmgrab PROPERTY LOCATION ${CMAKE_CURRENT_BINARY_DIR}/libfrmgrab.a )


    add_to_hal_include_dirs( ${CMAKE_CURRENT_SOURCE_DIR}/SDK/includes )

    add_to_hal_libraries( frmgrab )
#    add_to_hal_libraries( frmgrab z "-L/opt/local/lib/" jpeg png xml2 )
    add_to_hal_sources(
        EpiphanDriver.h EpiphanDriver.cpp EpiphanFactory.cpp ${SLAVA_FILES} ${V2U_FILES}
    )
endif()

