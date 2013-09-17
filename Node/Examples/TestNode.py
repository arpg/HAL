#!/usr/bin/python

# Script to test node correctness and performance.

# Test 1: launch and kill a bunch of nodes to test avahi
# Test 2: test publish subscribe
#         test name conflict
#         test bandwidth
#         test early sender death
#         test early receiver death (with multiple receivers)
# Test 3: test remote procedure calls
#         test ring of RPC calls with failure
#         test scatter gather of RPC calls with failure

import sys
import subprocess as proc

def main( argv ):
#    print 'hi %s'%argv 
#    proc.call(["ls", "-l"])
    n = 10;
    for i in range( ord('a'), ord('a')+n ):
        print chr(i),

if __name__ == "__main__":
    main( argv=sys.argv )

