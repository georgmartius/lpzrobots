FROM ubuntu:latest

RUN apt-get -y update && apt-get install -y \
    g++  make  automake libtool xutils-dev m4  libreadline-dev  libgsl0-dev \
    libglu-dev libgl1-mesa-dev freeglut3-dev  libopenscenegraph-dev \
    libqt4-dev libqt4-opengl libqt4-opengl-dev qt4-qmake  libqt4-qt3support gnuplot gnuplot-x11 \
    libncurses5-dev git
RUN cd /opt; git clone https://github.com/georgmartius/lpzrobots.git #redo

RUN echo ''\
'# configuration file for lpzrobots (automatically generated!)\n'\
'# # Where to install the simulator and utils\n'\
'PREFIX=/usr/local\n'\
'#\n'\
'# # user or developement installation\n'\
'# # Options:\n'\
'# #  DEVEL: only install the utility functions,\n'\
'# #   which is useful for development on the simulator\n'\
'# #  USER: install also the ode_robots and selforg libaries and include files\n'\
'# #   this is recommended for users\n'\
'TYPE=USER\n'\
>>/opt/lpzrobots/Makefile.conf

RUN cd /opt/lpzrobots; make all

RUN echo '[ ! -z "$TERM" -a -r /etc/motd ] && cat /etc/issue && cat /etc/motd' \
    >> /etc/bash.bashrc \
    ; echo ""\
"##################################################\n"\
"#          lpzrobots install directory:          #\n"\
"#                /opt/lpzrobots                  #\n"\
"#                                                #\n"\
"#      More information can be found under:      #\n"\
"#    https://github.com/georgmartius/lpzrobots   #\n"\
"##################################################\n"\
>>/etc/motd

RUN echo '\n\
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH'\
 >> /root/.bashrc
