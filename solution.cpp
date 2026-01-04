#include <array>
#include <asio.hpp>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <exception>
#include <functional>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

// To allow graceful shutdown on SIGINT.
std::atomic<bool> stopExecution{false};

void sigintHandler(int) { stopExecution = true; }

class ObjectParsingException : public std::runtime_error {
  using std::runtime_error::runtime_error;
};

typedef std::array<uint8_t, 3> color;

constexpr color RED{0x5B, 0x31, 0x6D}, YELLOW{0x5B, 0x33, 0x6D},
  BLUE{0x5B, 0x34, 0x6D};

constexpr std::pair<int32_t, int32_t> POI{150, 150};

float getDistanceFromPOI(int32_t x, int32_t y) {
  return std::pow(
    std::pow(x - POI.first, 2) + std::pow(y - POI.second, 2), 0.5
  );
}

class Object {
public:
  Object(int64_t, int32_t, int32_t, uint8_t);
  Object &operator=(Object &);

  int64_t getId() const;
  void outputObjectRepresentation() const;

private:
  const int64_t id;
  int32_t x;
  int32_t y;
  uint8_t type;
  uint8_t category;
  float distanceFromPOI;

  color getColor() const;
  static uint8_t getCategoryByType(uint8_t);
};

Object::Object(int64_t id, int32_t x, int32_t y, uint8_t type) :
    id{id},
    x{x},
    y{y},
    type{type},
    category{Object::getCategoryByType(type)},
    distanceFromPOI{getDistanceFromPOI(x, y)} {}

Object &Object::operator=(Object &copiedObject) {
  this->x = copiedObject.x;
  this->y = copiedObject.y;
  this->type = copiedObject.type;
  this->category = copiedObject.category;
  this->distanceFromPOI = copiedObject.distanceFromPOI;
  return *this;
}

int64_t Object::getId() const { return this->id; }

void Object::outputObjectRepresentation() const {
  std::cout.write(reinterpret_cast<const char *>(&this->id), 8);
  std::cout.write(reinterpret_cast<const char *>(&this->x), 4);
  std::cout.write(reinterpret_cast<const char *>(&this->y), 4);
  std::cout.write(reinterpret_cast<const char *>(&this->type), 1);
  color objectColor{this->getColor()};
  for (auto x : objectColor) {
    std::cout.write(reinterpret_cast<const char *>(&x), 1);
  }
}

color Object::getColor() const {
  if (this->category == 2) {
    if (this->distanceFromPOI >= 100)
      return YELLOW;
    return RED;
  } else {
    if (this->type == 1) {
      if (this->distanceFromPOI >= 75)
        return BLUE;
      else if (this->distanceFromPOI >= 50)
        return YELLOW;
      return RED;
    } else {
      if (this->distanceFromPOI >= 50)
        return BLUE;
      return YELLOW;
    }
  }
}

uint8_t Object::getCategoryByType(uint8_t type) { return (type < 3) ? 1 : 2; }

void clientThreadFunction(
  std::unordered_map<int64_t, Object> &objectMap, std::mutex &mutex
) {
  const auto referenceTime = std::chrono::steady_clock::now();

  std::unordered_map<int64_t, Object> temporaryMap;

  while (!stopExecution) {
    {
      std::lock_guard<std::mutex> lock(mutex);
      temporaryMap = objectMap;
    }

    const uint32_t fe00{0xFE00};
    const long unsigned int mapSize{temporaryMap.size()};
    std::cout.write(reinterpret_cast<const char *>(&fe00), 4);
    std::cout.write(reinterpret_cast<const char *>(&mapSize), 4);
    for (auto it{temporaryMap.begin()}; it != temporaryMap.end(); ++it)
      it->second.outputObjectRepresentation();
    std::cout.flush();

    const auto currentTime = std::chrono::steady_clock::now();
    // This truncates (i.e. floors) the time difference to the nearest
    // millisecond.
    const uint64_t millisecondsSinceReferenceTime =
      std::chrono::duration_cast<std::chrono::milliseconds>(
        currentTime - referenceTime
      )
        .count();
    const uint64_t currentCycle = millisecondsSinceReferenceTime / 5000;
    const int timeWithinCycle = millisecondsSinceReferenceTime % 5000;
    const int padding =
      (timeWithinCycle < 1667)
        ? 1667
        : ((timeWithinCycle < 3333) ? 3333 : 5000);
    std::this_thread::sleep_until(
      referenceTime +
      std::chrono::duration<uint64_t, std::milli>{5000 * currentCycle} +
      std::chrono::duration<int, std::milli>{padding}
    );
  }
}

int64_t getIntegerValueFromKeyValueString(const std::string &key_value_string) {
  return std::stoll(key_value_string.substr(
    key_value_string.find("=") + 1, key_value_string.length()
  ));
}

Object objectFromString(std::string &keyValuePairsString) {
  try {
    std::vector<int64_t> values;
    size_t position;
    while ((position = keyValuePairsString.find(";")) != std::string::npos) {
      values.push_back(getIntegerValueFromKeyValueString(
        keyValuePairsString.substr(0, position)
      ));
      keyValuePairsString.erase(0, position + 1);
    }
    values.push_back(getIntegerValueFromKeyValueString(keyValuePairsString));

    return Object(values[0], values[1], values[2], values[3]);
  } catch (const std::exception &e) {
    throw ObjectParsingException(e.what());
  }
}

int main() {
  std::unordered_map<int64_t, Object> objectMap;

  std::mutex mutex;

  std::signal(SIGINT, sigintHandler);

  std::thread clientThread(
    clientThreadFunction, std::ref(objectMap), std::ref(mutex)
  );

  asio::ip::tcp::iostream input("localhost", "5463");
  for (std::string str; std::getline(input, str);) {
    try {
      Object temporaryObject(objectFromString(str));
      std::lock_guard<std::mutex> lock(mutex);
      objectMap.insert_or_assign(temporaryObject.getId(), temporaryObject);
    } catch (const ObjectParsingException &e) {
      std::clog << "Failed to parse a line! Message: " << e.what() << std::endl;
    }
    if (stopExecution)
      break;
  }
  clientThread.join();

  return SIGINT;
}
