#include "EncCounter.h"


EncCounter::EncCounter(int enc_a, int enc_b){
    enc_A   = enc_a;
    enc_B   = enc_b;
    enc_val = 0;
    v_a     = false;
    v_b     = false;
    v_A     = false;
    v_B     = false;
    pinMode(enc_A, INPUT);
    pinMode(enc_B, INPUT);
    v_a = digitalRead(enc_A);
    v_b = digitalRead(enc_B);
    old_enc_val = 0;
}

void EncCounter::updt_enc(){
  v_A = v_a;   
  v_B = v_b;
  v_a = digitalRead(enc_A);
  v_b = digitalRead(enc_B);  
}

int EncCounter::counter(){
  /* conter comparres old values of encoder inputs and new ones
  using simplified conditions determined is direction */
  updt_enc();
  if ((v_A || v_b) && (!v_B || v_a) && 
      (!v_A || !v_b) && (v_B || !v_a)){
        enc_val += 1;
      }
  else if((v_B || v_a) && (v_A || !v_b) &&
      (!v_B || !v_a) && (!v_A || v_b)){
        enc_val -= 1;
  }
  return enc_val;
}

int EncCounter::calc_diff(){
  /*calculate number of impulses since last check*/
  enc_val = counter();
  int difference = enc_val - old_enc_val;
  old_enc_val = enc_val;
  return difference;
}
