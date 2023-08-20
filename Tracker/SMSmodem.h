#include <TinyGSM.h>


class SMSTinyGsm : public TinyGsm {
	
 public:
  explicit SMSTinyGsm(Stream& stream) :TinyGsm(stream) {
  }

};
