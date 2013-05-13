#include "ViewerGui.h"

/////////////////////////////////////////////////////////////////////////////
int main( int argc, char** argv )
{
    // Create GUI window
    ViewerGui gui("Beaver", 1024, 768);

    // Init GUI
    gui.Init(argc, argv);

    // Start GUI thread
    gui.Run();

    return 0;
}


