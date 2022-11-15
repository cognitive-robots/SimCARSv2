
#include <ori/simcars/geometry/trig_buff.hpp>

#include <chrono>
#include <random>
#include <cmath>
#include <iostream>

#define ARRAY_SIZE 1000000
#define GENERATED_NUM_DEMONINATOR 10e-3

using namespace ori::simcars::geometry;
using namespace std::chrono;

int main()
{
    TrigBuff const *trig_buff = TrigBuff::init_instance(2000, AngleType::RADIANS);

    time_point<high_resolution_clock> start_time;
    microseconds time_elapsed;

    FP_DATA_TYPE *input_array = new FP_DATA_TYPE[ARRAY_SIZE];
    FP_DATA_TYPE *output_array = new FP_DATA_TYPE[ARRAY_SIZE];

    std::minstd_rand generator(system_clock::now().time_since_epoch().count());

    size_t i;
    for (i = 0; i < ARRAY_SIZE; ++i)
    {
        input_array[i] = generator() / GENERATED_NUM_DEMONINATOR;
    }


    start_time = high_resolution_clock::now();

    for (i = 0; i < ARRAY_SIZE; ++i)
    {
        output_array[i] = sin(input_array[i]);
    }

    time_elapsed = duration_cast<microseconds>(high_resolution_clock::now() - start_time);
    std::cout << "Normal Sin: " << time_elapsed.count() << " us" << std::endl;


    start_time = high_resolution_clock::now();

    for (i = 0; i < ARRAY_SIZE; ++i)
    {
        output_array[i] = trig_buff->get_sin(input_array[i]);
    }

    time_elapsed = duration_cast<microseconds>(high_resolution_clock::now() - start_time);
    std::cout << "Trig Buff Sin: " << time_elapsed.count() << " us" << std::endl;


    start_time = high_resolution_clock::now();

    for (i = 0; i < ARRAY_SIZE; ++i)
    {
        output_array[i] = cos(input_array[i]);
    }

    time_elapsed = duration_cast<microseconds>(high_resolution_clock::now() - start_time);
    std::cout << "Normal Cos: " << time_elapsed.count() << " us" << std::endl;


    start_time = high_resolution_clock::now();

    for (i = 0; i < ARRAY_SIZE; ++i)
    {
        output_array[i] = trig_buff->get_cos(input_array[i]);
    }

    time_elapsed = duration_cast<microseconds>(high_resolution_clock::now() - start_time);
    std::cout << "Trig Buff Cos: " << time_elapsed.count() << " us" << std::endl;


    start_time = high_resolution_clock::now();

    for (i = 0; i < ARRAY_SIZE; ++i)
    {
        output_array[i] = tan(input_array[i]);
    }

    time_elapsed = duration_cast<microseconds>(high_resolution_clock::now() - start_time);
    std::cout << "Normal Tan: " << time_elapsed.count() << " us" << std::endl;


    start_time = high_resolution_clock::now();

    for (i = 0; i < ARRAY_SIZE; ++i)
    {
        output_array[i] = trig_buff->get_tan(input_array[i]);
    }

    time_elapsed = duration_cast<microseconds>(high_resolution_clock::now() - start_time);
    std::cout << "Trig Buff Tan: " << time_elapsed.count() << " us" << std::endl;

    delete[] input_array;
    delete[] output_array;

    TrigBuff::destroy_instance();
}
