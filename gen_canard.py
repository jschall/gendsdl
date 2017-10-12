import sys
import os
from glob import glob

import genmsg.template_tools

msg_template_map = { 'templates/msg.h.template':'@NAME@.h' }
srv_template_map = { 'templates/srv.h.template':'@NAME@.h' }

if __name__ == "__main__":
    from optparse import OptionParser
    parser = OptionParser("[options] <root_directory>")
    parser.add_option("-o", dest='outdir',
                      help="directory in which to place output files")
    parser.add_option("-e", dest='emdir',
                      help="directory containing template files",
                      default=sys.path[0])

    (options, argv) = parser.parse_args(sys.argv)

    if( not options.outdir or not options.emdir):
        parser.print_help()
        exit(-1)
    file_list = [os.path.relpath(y, os.path.dirname(argv[1])) for x in os.walk(argv[1]) for y in glob(os.path.join(x[0], '*.uavcan'))]
    includepath = [argv[1]]
    for file_name in file_list:
        if len(file_name) > 1:
            genmsg.template_tools.generate_from_file(os.path.dirname(argv[1]), file_name, os.path.dirname(file_name), options.outdir, options.emdir, includepath, msg_template_map, srv_template_map)

