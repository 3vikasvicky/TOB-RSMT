#include <iostream>
#include <string.h>

using namespace std;

int main(){
    char a[100] = "abcd";
    cout << strlen(a)<< endl;
    *(a+strlen(a)) = 'e';
    *(a+strlen(a)+1) = '\0';
    cout<< a<< endl<< strlen(a)<< endl;


}
