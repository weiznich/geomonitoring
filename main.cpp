#include <iostream>
#include <vector>

int main(int argc,char **argv){
    std::cout<<"Hello world"<<std::endl;
    //test some c++11 stuff
    //if this compiles the enviroment is setup in the right way
    //output should be all numbers from 0 to 9 
    std::vector<int> numbers;

    for(int i=0;i<10;i++)
        numbers.push_back(i);

    for(auto number:numbers){
        std::cout<<number<<std::endl;
    }
    return 1;

}
