set -e

cd ../../Fido/
make
make lib

cd ../Research/Experiments
make clean
make
./experiments.o
