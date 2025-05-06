wget -q https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
tar -xzf eigen-3.4.0.tar.gz

g++ -std=c++17 -I eigen-3.4.0 twobody_spring_damper.cpp -o twobody_spring_damper
./twobody_spring_damper