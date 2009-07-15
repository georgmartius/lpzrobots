# Run this in a simulation folder to check for memory mistakes

valgrind --tool=memcheck --leak-check=yes --suppressions=ode_robots.supp  ./start -nographics -noshadow
