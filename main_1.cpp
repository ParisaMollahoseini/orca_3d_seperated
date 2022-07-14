#include <iostream>
#include <omp.h>
int main()
{
    std::cout<< omp_get_num_threads();
    #pragma omp parallel
    {
        std::cout << "Hello World" << std::endl;
    }
    return 0;
}