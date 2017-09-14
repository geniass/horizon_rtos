#include "HeaveFilter.h"

static float filter_taps[HEAVEFILTER_TAP_NUM] = {
  0.005306608379116704,
  0.0019635301634892016,
  0.00043948190029313196,
  -0.002688970903532857,
  -0.007183335736691441,
  -0.01226469711016815,
  -0.016663152946643705,
  -0.01872379142090098,
  -0.016751503537724095,
  -0.009357588613583965,
  0.004154005309683565,
  0.0235007051423076,
  0.04731516581727753,
  0.07323010053396799,
  0.09820702392515775,
  0.11901897421144955,
  0.132816163238448,
  0.1376472220478503,
  0.132816163238448,
  0.11901897421144955,
  0.09820702392515775,
  0.07323010053396799,
  0.04731516581727753,
  0.0235007051423076,
  0.004154005309683565,
  -0.009357588613583965,
  -0.016751503537724095,
  -0.01872379142090098,
  -0.016663152946643705,
  -0.01226469711016815,
  -0.007183335736691441,
  -0.0026889709035328637,
  0.00043948190029313196,
  0.0019635301634892016,
  0.005306608379116704
};

void HeaveFilter_init(HeaveFilter* f) {
  int i;
  for(i = 0; i < HEAVEFILTER_TAP_NUM; ++i)
    f->history[i] = 0;
  f->last_index = 0;
}

void HeaveFilter_put(HeaveFilter* f, float input) {
  f->history[f->last_index++] = input;
  if(f->last_index == HEAVEFILTER_TAP_NUM)
    f->last_index = 0;
}

float HeaveFilter_get(HeaveFilter* f) {
  float acc = 0;
  int index = f->last_index, i;
  for(i = 0; i < HEAVEFILTER_TAP_NUM; ++i) {
    index = index != 0 ? index-1 : HEAVEFILTER_TAP_NUM-1;
    acc += f->history[index] * filter_taps[i];
  };
  return acc;
}
