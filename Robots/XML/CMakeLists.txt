set( XML_HDRS rg_tinyxml.h )
set( XML_SRCS rg_tinyxml.cpp rg_tinyxmlparser.cpp rg_tinyxmlerror.cpp )
add_library( xml ${XML_SRCS} ${XML_HDRS}  )
