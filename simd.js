/*Copyright (c) 2017 Dominik Homberger

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

https://webpjs.appspot.com

Port from libwebp.git

libwebpjs SIMD Addon dominikhlbg@gmail.com
*/

//------------------------------------------------------------------------------
//

function get_16_bytes(dst,dst_off) {//uint8x16 uint8_t* 
  var i=0;
  var arr=new Uint8Array(16)
  for(i=0;i<16;++i)
  arr[i]=dst[dst_off+i];
  var a=SIMD.Uint8x16.load(arr,0);//uint8x16
  return a;
}

function get_8_bytes(dst,dst_off) {//uint8x16 uint8_t* 
  var i=0;
  var arr=new Uint8Array(16)
  for(i=0;i<8;++i)
  arr[i]=dst[dst_off+i];
  var a=SIMD.Uint8x16.load(arr,0);//uint8x16
  return a;
}

function splat_uint8(val) {//uint8x16 uint32_t 
  var a=SIMD.Uint8x16.splat(val);//uint8x16
  return a;
}

//------------------------------------------------------------------------------
// Luma 16x16

function Put16(v, dst, dst_off) {//uint8_t uint8_t* 
  var j=0;var i=0;//int
  var values = splat_uint8(v);//const uint8x16
  for (j = 0; j < 16; ++j) {
    for(i=0;i<16;++i)
      dst[dst_off +i]=SIMD.Uint8x16.extractLane(values,i);
      dst_off += BPS;
  }
}

function VE16(dst,dst_off) {//uint8_t* 
  var top = get_16_bytes(dst,dst_off - BPS);//const uint8x16
  var j=0;var i=0;//int
  for (j = 0; j < 16; ++j) {
    for(i=0;i<16;++i)
	  dst[dst_off + j * BPS +i]=SIMD.Uint8x16.extractLane(top,i);
  }
}

function HE16(dst,dst_off) {//uint8_t*      // horizontal
  var j=0;var i=0;//int
  for (j = 16; j > 0; --j) {
    var values = splat_uint8(dst[dst_off-1]);//const uint8x16
    for(i=0;i<16;++i)
	  dst[dst_off +i]=SIMD.Uint8x16.extractLane(values,i);
    dst_off += BPS;
  }
}

function add_horizontal_16(dst,dst_off) {//uint8_t* 
  var zero = SIMD.Uint8x16.splat(0);
  var a = get_16_bytes(dst,dst_off);//const uint8x16
  var _a_lbw = SIMD.Uint8x16.shuffle(
      a, zero, 0, 16, 1, 16, 2, 16, 3, 16, 4, 16, 5, 16, 6, 16, 7, 16);
  var _a_hbw = SIMD.Uint8x16.shuffle(
      a, zero, 8, 16, 9, 16, 10, 16, 11, 16, 12, 16, 13, 16, 14, 16, 15, 16);
  var sum_a = SIMD.Uint16x8.add(SIMD.Uint16x8.fromUint8x16Bits(_a_lbw), SIMD.Uint16x8.fromUint8x16Bits(_a_hbw));//const uint16x8
  var sum_b = SIMD.Uint16x8.shuffle(
      sum_a, sum_a, 4, 5, 6, 7, 4, 5, 6, 7);
  var sum_c = SIMD.Uint16x8.add(sum_a, sum_b);//const uint16x8
  var sum_d = SIMD.Uint16x8.shuffle(
      sum_c, sum_c, 2, 3, 2, 3, 2, 3, 2, 3);
  var sum_e = SIMD.Uint16x8.add(sum_c, sum_d);//const uint16x8
  var sum_f = SIMD.Uint16x8.shuffle(
      sum_e, sum_e, 1, 1, 1, 1, 1, 1, 1, 1);
  var sum_g = SIMD.Uint16x8.add(sum_e, sum_f);//const uint16x8
  return SIMD.Uint16x8.extractLane(sum_g,0) & 0xffff;
}

function DC16(dst,dst_off) {//uint8_t*     // DC
  var sum = add_horizontal_16(dst,dst_off - BPS);//const uint32_t
  var left = 0;//int
  var j;//int
  for (j = 0; j < 16; ++j) {
    left += dst[dst_off-1 + j * BPS];
  }
  {
    var DC = sum + left + 16;//const int
    Put16(DC >> 5, dst, dst_off);
  }
}

function DC16NoTop(dst,dst_off) {//uint8_t*    // DC with top samples not available
  var DC = 8;//int
  var j=0;//int
  for (j = 0; j < 16; ++j) {
    DC += dst[dst_off-1 + j * BPS];
  }
  Put16(DC >> 4, dst, dst_off);
}

function DC16NoLeft(dst,dst_off) {//uint8_t*   // DC with left samples not available
  var DC = 8 + add_horizontal_16(dst,dst_off - BPS);//const int
  Put16(DC >> 4, dst, dst_off);
}

function DC16NoTopLeft(dst,dst_off) {//uint8_t*   // DC with no top and left samples
  Put16(0x80, dst, dst_off);
}

//------------------------------------------------------------------------------
// Chroma

function add_horizontal_8(dst,dst_off) {//uint32_t uint8_t* 
  var zero = SIMD.Uint8x16.splat(0);
  var a = get_8_bytes(dst,dst_off);//const uint8x16
  var _a_lbw = SIMD.Uint8x16.shuffle(
      a, zero, 0, 16, 1, 16, 2, 16, 3, 16, 16, 16, 16, 16, 16, 16, 16, 16);
  var _a_hbw = SIMD.Uint8x16.shuffle(
      a, zero, 4, 16, 5, 16, 6, 16, 7, 16, 16, 16, 16, 16, 16, 16, 16, 16);
  var sum_a = SIMD.Uint16x8.add(SIMD.Uint16x8.fromUint8x16Bits(_a_lbw), SIMD.Uint16x8.fromUint8x16Bits(_a_hbw));//const uint16x8
  var sum_b = SIMD.Uint16x8.shuffle(
      sum_a, sum_a, 2, 3, 2, 3, 2, 3, 2, 3);
  var sum_c = SIMD.Uint16x8.add(sum_a, sum_b);//const uint16x8
  var sum_d = SIMD.Uint16x8.shuffle(
      sum_c, sum_c, 1, 1, 1, 1, 1, 1, 1, 1);
  var sum_e = SIMD.Uint16x8.add(sum_c, sum_d);//const uint16x8
  return SIMD.Uint16x8.extractLane(sum_e,0) & 0xffff;
}

function VE8uv(dst,dst_off) {//uint8_t*     // vertical
  var top = get_8_bytes(dst,dst_off - BPS);//const uint8x16
  var j=0;var i=0;//int
  for (j = 0; j < 8; ++j) {
    for(i=0;i<8;++i)
	  dst[dst_off + j * BPS +i]=SIMD.Uint8x16.extractLane(top,i);
    //memcpy(dst + j * BPS, &top, 8);
  }
}

function HE8uv(dst,dst_off) {//uint8_t*     // horizontal
  var j=0;var i=0;//int
  for (j = 8; j > 0; --j) {
    var values = splat_uint8(dst[dst_off-1]);//const uint8x16
    for(i=0;i<8;++i)
	  dst[dst_off +i]=SIMD.Uint8x16.extractLane(values,i);
    //memcpy(dst, &values, 8);
    dst_off += BPS;
  }
}

// helper for chroma-DC predictions
function Put8x8uv(v, dst, dst_off) {//uint8_t uint8_t* 
  var j=0;var i=0;//int
  var values = splat_uint8(v);//const uint8x16
  for (j = 0; j < 8; ++j) {
    for(i=0;i<8;++i)
	  dst[dst_off + j * BPS +i]=SIMD.Uint8x16.extractLane(values,i);
    //memcpy(dst + j * BPS, &values, 8);
  }
}

function DC8uv(dst,dst_off) {//uint8_t*      // DC
  var left = 0;//int
  var j=0;//int
  var sum = add_horizontal_8(dst,dst_off - BPS);//const uint32_t
  for (j = 0; j < 8; ++j) {
    left += dst[dst_off-1 + j * BPS];
  }
  {
    var DC = sum + left + 8;//const int
    Put8x8uv(DC >> 4, dst, dst_off);
  }
}

function DC8uvNoLeft(dst,dst_off) {//uint8_t*    // DC with no left samples
  var DC = 4 + add_horizontal_8(dst,dst_off - BPS);//const uint32_t
  Put8x8uv(DC >> 3, dst, dst_off);
}

function DC8uvNoTop(dst,dst_off) {//uint8_t*   // DC with no top samples
  var dc0 = 4;//int
  var i=0;//int
  for (i = 0; i < 8; ++i) {
    dc0 += dst[dst_off-1 + i * BPS];
  }
  Put8x8uv(dc0 >> 3, dst, dst_off);
}

function DC8uvNoTopLeft(dst,dst_off) {//uint8_t*     // DC with nothing
  Put8x8uv(0x80, dst, dst_off);
}

//------------------------------------------------------------------------------
// Entry point

function VP8DspInitSIMDJS() {
  VP8PredLuma16[0] = DC16;
  VP8PredLuma16[2] = VE16;
  VP8PredLuma16[3] = HE16;
  VP8PredLuma16[4] = DC16NoTop;
  VP8PredLuma16[5] = DC16NoLeft;
  VP8PredLuma16[6] = DC16NoTopLeft;

  VP8PredChroma8[0] = DC8uv;
  VP8PredChroma8[2] = VE8uv;
  VP8PredChroma8[3] = HE8uv;
  VP8PredChroma8[4] = DC8uvNoTop;
  VP8PredChroma8[5] = DC8uvNoLeft;
  VP8PredChroma8[6] = DC8uvNoTopLeft;
}
VP8DspInitSIMDJS();
