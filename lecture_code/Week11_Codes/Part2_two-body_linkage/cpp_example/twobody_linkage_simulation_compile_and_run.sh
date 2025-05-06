wget -q https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
tar -xzf eigen-3.4.0.tar.gz

g++ -std=c++17 -I eigen-3.4.0 closed_chain_sim.cpp -o closed_chain_sim
./closed_chain_sim