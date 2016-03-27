set -e

cd ../../Fido/Software
make
make lib

cd ../../Research/Experiments
make clean
make
./experiments.o
