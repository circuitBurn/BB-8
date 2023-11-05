//#include <EEPROMex.h>
//
//class Offsets
//{
//  private:
//    float _roll;
//    float _pitch;
//
//    void writeToEEPROM()
//    {
//      EEPROM.writeFloat(0, _pitch);
//      EEPROM.writeFloat(4, _roll);
//    }
//
//    void loadFromEEPROM()
//    {
//      _pitch = EEPROM.readFloat(0);
//      _roll = EEPROM.readFloat(4);
//    }
//
//  public:
//    float roll() {
//      return _roll;
//    }
//    float pitch() {
//      return _pitch;
//    }
//
//    void initialize()
//    {
//      loadFromEEPROM();
//    }
//
//    void set(float pitch, float roll)
//    {
//      _roll = roll;
//      _pitch = pitch;
//      writeToEEPROM();
//    }
//};
