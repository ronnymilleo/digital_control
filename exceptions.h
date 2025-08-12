#pragma once

#include <stdexcept>
#include <string>

namespace DigitalControl
{

/**
 * Base exception class for digital control library
 */
class DigitalControlException : public std::runtime_error
{
  public:
    explicit DigitalControlException(const std::string &message) : std::runtime_error("DigitalControl: " + message)
    {
    }
};

/**
 * Exception thrown when invalid parameters are provided
 */
class InvalidParameterException : public DigitalControlException
{
  public:
    explicit InvalidParameterException(const std::string &message)
        : DigitalControlException("Invalid parameter: " + message)
    {
    }
};

/**
 * Exception thrown when numerical computation fails
 */
class NumericalException : public DigitalControlException
{
  public:
    explicit NumericalException(const std::string &message) : DigitalControlException("Numerical error: " + message)
    {
    }
};

/**
 * Exception thrown when controller configuration is invalid
 */
class ConfigurationException : public DigitalControlException
{
  public:
    explicit ConfigurationException(const std::string &message)
        : DigitalControlException("Configuration error: " + message)
    {
    }
};

} // namespace DigitalControl
