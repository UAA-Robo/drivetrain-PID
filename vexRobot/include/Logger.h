#pragma once
#include <Hardware.h>
#include <vex.h>

/// @brief    Used to initiate a log text/csv file and write data to the text file.
class Logger {
    
public:
    /// @brief     Opens file stream and writes column headers.
    /// @param hardware    Uses brain from Hardware.
    /// @param file_name    Name of file (including .txt/.csv).
    /// @param column_names    Vector of column names for data to store in each column.
    Logger(Hardware* hardware, std::string fileName, std::vector<std::string> column_names);

    /// @brief    Closes stream to file.
    ~Logger();

    /// @brief   Adds the passed in data to the file.
    /// @param row   Vector to add to the file as a new row, with each element corresponding to a 
    ///              column in the same order as the column_names initialized in the constructor.
    void add_data(std::vector<double> row);

    /// @brief Converts anything to a string (VEX doesn't have std::to_string)
    /// @param value Any value to convert t a string.
    /// @return The value as a string.
    template <typename T>
    static std::string to_string(T value);


private:
    Hardware* hw;
    std::ofstream data_log;
    std::string file;
};


template <typename T>
std::string Logger::to_string(T value)
{
    std::ostringstream os ;
    os << value ;
    return os.str() ;
}

