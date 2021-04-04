#include <string>

class Car
{
public:
    Car(std::string color, int max_speed, std::string brand);
    ~Car();

    std::string brand_, color_;

    int max_speed_;

    void printStats();

    static void whatIsThisClass();
private:
};

Car::Car(std::string color, int max_speed, std::string brand)
{
    color_ = color;
    max_speed_ = max_speed;
    brand_ = brand;
}

void Car::printStats()
{
    printf("This is a %s car that is %s and can go %d mph.\n", brand_, color_, max_speed_);
}

void Car::whatIsThisClass()
{
    printf("This class represents a car.\n");

    // we can't touch brand_, color_, or max_speed_
}

int main(int argc, char* argv[])
{
    Car* redToyota = new Car("red", 120, "Toyota");
    Car* lightspeedBlueHonda = new Car("blue", 100000000, "Honda");
    
    redToyota->printStats();
    lightspeedBlueHonda->printStats();

    Car::whatIsThisClass();

    return 0;
}