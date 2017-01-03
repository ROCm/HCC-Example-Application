#include <vector>
#include <iostream>
#include <string>
#include <locale>
#include <codecvt>

#include <hc.hpp>

std::string getDeviceName(const hc::accelerator& _acc)
{
  std::wstring_convert<std::codecvt_utf8<wchar_t>, wchar_t> converter;
  std::string value = converter.to_bytes(_acc.get_description());
  return value;
}

void listDevices(void)
{
  // Get number of devices
  std::vector<hc::accelerator> accs = hc::accelerator::get_all();

  // Print device names
  if (accs.empty())
  {
    std::cerr << "No devices found." << std::endl;
  }
  else
  {
    std::cout << std::endl;
    std::cout << "Devices:" << std::endl;
    for (int i = 0; i < accs.size(); i++)
    {
      std::cout << i << ": " << getDeviceName(accs[i]) << std::endl;
    }
    std::cout << std::endl;
  }
}

int main(int argc, char* argv[])
{
    std::cout << "************************************************" << std::endl;
    std::cout << "              list devices " << std::endl;
    std::cout << "************************************************" << std::endl;
    std::cout << std::endl;

    listDevices();
    
    return 0;
}

