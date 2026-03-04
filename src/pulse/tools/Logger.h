#pragma once

#include <Arduino.h>
#include <cstdarg>  // For va_list, va_start, and va_end

namespace pulsewire {

/**
 * @brief Simple logger class for debugging and logging messages.
 *
 * This class provides a lightweight logging utility for printing messages
 * to the serial console. It supports different log levels and can be
 * enabled or disabled globally. It also supports formatted messages using
 * variadic arguments, similar to printf.
 */
class Logger {
 public:
  enum LogLevel {
    LOG_LEVEL_NONE,
    LOG_LEVEL_ERROR,
    LOG_LEVEL_WARNING,
    LOG_LEVEL_INFO,
    LOG_LEVEL_DEBUG
  };

  /**
   * @brief Set the global log level.
   * @param level The log level to set.
   */
  static void setLogLevel(LogLevel level) { _logLevel = level; }

  /**
   * @brief Log an error message with formatting.
   * @param format The format string (printf-style).
   * @param ... Variadic arguments for the format string.
   */
  static void error(const char* format, ...) {
    if (_logLevel >= LOG_LEVEL_ERROR) {
      Serial.print("[ERROR] ");
      va_list args;
      va_start(args, format);
      logFormatted(format, args);
      va_end(args);
    }
  }

  /**
   * @brief Log a warning message with formatting.
   * @param format The format string (printf-style).
   * @param ... Variadic arguments for the format string.
   */
  static void warning(const char* format, ...) {
    if (_logLevel >= LOG_LEVEL_WARNING) {
      Serial.print("[WARNING] ");
      va_list args;
      va_start(args, format);
      logFormatted(format, args);
      va_end(args);
    }
  }

  /**
   * @brief Log an informational message with formatting.
   * @param format The format string (printf-style).
   * @param ... Variadic arguments for the format string.
   */
  static void info(const char* format, ...) {
    if (_logLevel >= LOG_LEVEL_INFO) {
      Serial.print("[INFO] ");
      va_list args;
      va_start(args, format);
      logFormatted(format, args);
      va_end(args);
    }
  }

  /**
   * @brief Log a debug message with formatting.
   * @param format The format string (printf-style).
   * @param ... Variadic arguments for the format string.
   */
  static void debug(const char* format, ...) {
    if (_logLevel >= LOG_LEVEL_DEBUG) {
      Serial.print("[DEBUG] ");
      va_list args;
      va_start(args, format);
      logFormatted(format, args);
      va_end(args);
    }
  }

  /**
   * @brief Set the Output object   
   * 
   * @param print 
   */
  static void setOutput(Print& print) { output = &print; }

 private:
  static LogLevel _logLevel;
  static Print* output;

  /**
   * @brief Helper function to log a formatted message.
   * @param format The format string (printf-style).
   * @param args The variadic arguments list.
   */
  static void logFormatted(const char* format, va_list args) {
    char buffer[256];  // Adjust the buffer size as needed
    vsnprintf(buffer, sizeof(buffer), format, args);
    output->println(buffer);
  }
};

// Initialize the static member variable
Logger::LogLevel Logger::_logLevel = Logger::LOG_LEVEL_NONE;
Print* Logger::output = &Serial;

}  // namespace pulsewire

#define TRACE() Logger::debug("%s:%d - %s", __FILE__, __LINE__, __func__)
//#define TRACE() 