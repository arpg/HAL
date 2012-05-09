# 1) Build the library
include_directories( SDK/includes )
set( DVI2PCI_DRIVER_HEADERS Dvi2PciDriver.h )
set( DVI2PCI_DRIVER_SOURCES Dvi2PciDriver.cpp )
add_library( Dvi2PciDriver ${DVI2PCI_DRIVER_HEADERS} ${DVI2PCI_DRIVER_SOURCES} )

# copy auxilary library to build directory
add_library( v2u SDK/v2u_lib/src/v2u_bmp.c
                SDK/v2u_lib/src/v2u_jpeg.c
                SDK/v2u_lib/src/v2u_pal.c
                SDK/v2u_lib/src/v2u_png.c
                SDK/v2u_lib/src/v2u_sys.c
                SDK/v2u_lib/src/v2u_util.c )

add_library( slava SDK/slib/src/s_base32.c
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

add_to_rpg_libraries( slava )

if(APPLE)
	file( COPY SDK/libfrmgrab/macosx/libfrmgrab.a DESTINATION ${CMAKE_CURRENT_BINARY_DIR} )
else()
	file( COPY SDK/libfrmgrab/linux/x86_64/libfrmgrab.a DESTINATION ${CMAKE_CURRENT_BINARY_DIR} )
endif(APPLE)

add_custom_target( frmgrab )
set_property( TARGET frmgrab PROPERTY LOCATION ${CMAKE_CURRENT_BINARY_DIR}/libfrmgrab.a )
add_to_rpg_libraries( frmgrab )

# 2) Tell RPG about this library:
add_to_rpg_include_dirs( ${CMAKE_CURRENT_SOURCE_DIR} )
add_to_rpg_include_dirs( ${CMAKE_CURRENT_SOURCE_DIR}/includes )
add_to_rpg_libraries( Dvi2PciDriver )
add_to_rpg_libraries( v2u )
add_to_rpg_link_directories( ${CMAKE_CURRENT_BINARY_DIR} )

# 3) And make sure we are able to install it correctly:
install( FILES frmgrab DESTINATION ${CMAKE_INSTALL_PREFIX}/lib )
install( TARGETS Dvi2PciDriver ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib )
install( FILES ${DVI2PCI_DRIVER_HEADERS}
         DESTINATION
         ${CMAKE_INSTALL_PREFIX}/include/RPG/Devices/Camera/Drivers/Dvi2Pci )

