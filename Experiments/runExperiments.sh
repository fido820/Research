set -e

cd ../../Fido/
make
make lib

cd ../FidoResearch/Experiments
make clean
make
./experiments.o
