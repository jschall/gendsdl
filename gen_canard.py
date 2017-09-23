import sys
import os

px4_tools_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(px4_tools_dir + "/modules/genmsg/src")
sys.path.append(px4_tools_dir + "/modules/gencpp/src")

import genmsg.template_tools

msg_template_map = { 'msg.h.template':'@NAME@.h' }
srv_template_map = { 'srv.h.template':'@NAME@.h' }

if __name__ == "__main__":
    genmsg.template_tools.generate_from_command_line_options(sys.argv,
                                                             msg_template_map,
                                                             srv_template_map)