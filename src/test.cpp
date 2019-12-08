#include <iostream>
using namespace std;

union Convert {
    unsigned int byte[4];
    float real;
};

int main() {
    cout << "Hola" <<endl;
    float valor = 3.589;
    
    int bytes[sizeof(float)];
    
    *(float*)(bytes) = valor;
    
    for(int i = 0; i < sizeof(float); i++) {
        cout <<bytes[i] <<"\t";
    } cout <<endl;
    
    Convert convert;
    
    convert.byte[0] = bytes[0];
    convert.byte[1] = bytes[1];
    convert.byte[2] = bytes[2];
    convert.byte[3] = bytes[3];
    
    cout << "Float is : " <<convert.real <<endl;
    
    convert.real = valor;
    
    for(int i = 0; i < sizeof(float); i++) {
        cout <<convert.byte[i] <<"\t";
    } cout <<endl;
    
    cout << "Float is : " <<convert.real <<endl;
    return 0;
}