#ifndef EncCounter_h
#define EncCounter_h
#include <Arduino.h>


class EncCounter{
  private:
    int enc_A;
    int enc_B;
    int enc_val;
    bool v_a;
    bool v_b;
    bool v_A;
    bool v_B;
    void updt_enc();
    int old_enc_val;

  public:
    EncCounter(int enc_a, int enc_b);
    int counter();
    int calc_diff();
};

#endif