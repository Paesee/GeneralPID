vazio

gcc -c src\GeneralPID.c -o object\GeneralPID.o
gcc -shared -o lib\libGeneralPID.dll object\GeneralPID.o