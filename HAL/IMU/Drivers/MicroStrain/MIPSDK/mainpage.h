/////////////////////////////////////////////////////////////////////////////
//
//! @file    mainpage.h 
//! @author  Nathan Miller
//! @version 1.0
//! @mainpage 
//! @section intro Introduction
//!
//! This documentation provides detailed information for the core MIP Interface as well\n
//! as all supported MIP command and data descriptors.  MIP-enabled devices typically\n
//! only support a subset of available MIP commands.  Please consult your device's DCP\n
//! for a list of supported commands.\n\n
//! 
//! Please use the <A HREF="annotated.html">Data Structures Tab</A> at the top of the page for details on the\n
//! availabe MIP data structures.\n\n
//! 
//! Please use the <A HREF="files.html">Files Tab</A> at the top of the page for the MIP function\n
//! documentation.\n
//!
//! \n\n
//! The C MIP SDK has been designed with portability in mind.  The target-specific functions have all been grouped\n
//! into a single source file, which must be edited by the user to implement hardware-level calls (e.g. serial port open, close, read, write, etc.)\n
//! on their target platform.  The remaining files are target-agnostic, and can be compiled on a wide range of platforms.\n
//! This structure allows the user to develop across several platforms easily.\n\n
//!
//! The SDK include a few key components:  a MIP parser, a callback interface for data packets, functions for assembling and processing MIP packets,\n
//! and high-level functions for performing descriptor-set-specific commands and byte-swapping MIP data.  
//!
//!
//! \n\n
//! @section support Support 
//!
//! Please visit the <A HREF="http://www.microstrain.com">Microstrain Homepage</A>
//! to contact support or access the latest support documentation for your device(s).
//! \n\n\n
//!
//! @section license License
//! THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING\n
//! CUSTOMERS WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER\n 
//! FOR THEM TO SAVE TIME. AS A RESULT, MICROSTRAIN SHALL NOT BE HELD LIABLE\n
//! FOR ANY DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY\n
//! CLAIMS ARISING FROM THE CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY\n 
//! CUSTOMERS OF THE CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH\n
//! THEIR PRODUCTS.\n
//
/////////////////////////////////////////////////////////////////////////////

