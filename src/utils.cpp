#include "utils.h"
#include <iostream>

// Function to add two integers
int add(int a, int b)
{
    return a + b;
}

// Function to multiply two doubles
double multiply(double x, double y)
{
    return x * y;
}

// Function to create a greeting message
std::string greet(const std::string &name)
{
    return "Hello, " + name + "!";
}

// Function to print an array
void printArray(const int arr[], int size)
{
    std::cout << "Array: [";
    for (int i = 0; i < size; i++)
    {
        std::cout << arr[i];
        if (i < size - 1)
        {
            std::cout << ", ";
        }
    }
    std::cout << "]" << std::endl;
}
