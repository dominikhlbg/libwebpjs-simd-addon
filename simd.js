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

function get_8_bytes_fromInt16bits(dst,dst_off) {//uint8x16 uint8_t* 
  var i=0;
  var arr=new Int16Array(16)
  for(i=0;i<8;++i)
  arr[i]=dst[dst_off+i];
  var a=SIMD.Uint8x16.load(arr,0);//uint8x16
  return a;
}

function splat_uint8(val) {//uint8x16 uint32_t 
  var a=SIMD.Uint8x16.splat(val);//uint8x16
  return a;
}

function cvt32_to_128(x) {//uint32x4 uint32_t 
  var i=0;
  var arr=new Uint32Array(4)
  arr[0]=x;
  var value = SIMD.Uint32x4.load(arr,0);// (uint32x4){0};//uint32x4
  return value;
}

function _unpacklo_epi8(a, b) {//int16x8 const int8x16 const int8x16 
  return SIMD.Int16x8.fromInt8x16Bits(SIMD.Int8x16.shuffle(a, b, 0, 16, 1, 17, 2, 18, 3, 19, 4, 20, 5, 21,
                                 6, 22, 7, 23));
}

function _unpacklo_epi16(a, b) {//int32x4 const int16x8 const int16x8 
  return SIMD.Int32x4.fromInt16x8Bits(SIMD.Int16x8.shuffle(a, b, 0, 8, 1, 9, 2, 10, 3, 11));
}

function _unpackhi_epi16(a, b) {//int32x4 const int16x8 const int16x8 
  return SIMD.Int32x4.fromInt16x8Bits(SIMD.Int16x8.shuffle(a, b, 4, 12, 5, 13, 6, 14, 7, 15));
}

function _unpacklo_epi32(a, b) {//int32x4 const int32x4 const int32x4 
  return SIMD.Int32x4.shuffle(a, b, 0, 4, 1, 5);
}

function _unpackhi_epi32(a, b) {
  return SIMD.Int32x4.shuffle(a, b, 2, 6, 3, 7);
}

function _unpacklo_epi64(a, b) {
  return SIMD.Int32x4.shuffle(a, b, 0, 1, 4, 5);
}

function _unpackhi_epi64(a, b) {
  return SIMD.Int32x4.shuffle(a, b, 2, 3, 6, 7);
}

function _mulhi_int16x8(in_, k) {//int16x8 int16x8 int32x4 
  var zero = SIMD.Int16x8(0, 0, 0, 0, 0, 0, 0, 0);
  var sixteen = SIMD.Int32x4(16, 16, 16, 16);
  // Put in upper 16 bits so we can preserve the sign
  var in_lo =
      SIMD.Int32x4.fromInt16x8Bits(SIMD.Int16x8.shuffle(in_, zero, 8, 0, 8, 1, 8, 2, 8, 3));
  var in_hi =
      SIMD.Int32x4.fromInt16x8Bits(SIMD.Int16x8.shuffle(in_, zero, 8, 4, 8, 5, 8, 6, 8, 7));
  var _lo = SIMD.Int32x4.mul(SIMD.Int32x4.shiftRightByScalar(in_lo, 16), k);//sixteen
  var _hi = SIMD.Int32x4.mul(SIMD.Int32x4.shiftRightByScalar(in_hi, 16), k);//sixteen
  // only keep the upper 16 bits
  var res = SIMD.Int16x8.shuffle(
      SIMD.Int16x8.fromInt32x4Bits(_lo), SIMD.Int16x8.fromInt32x4Bits(_hi), 1, 3, 5, 7, 9, 11, 13, 15);
  return res;
}

function int16x8_to_uint8x16_sat(x) {//uint8x16 int16x8 
  var k00ff00ff =
      SIMD.Uint8x16(-1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0);
  var fifteen = SIMD.Int16x8(15, 15, 15, 15, 15, 15, 15, 15);
  var zero = SIMD.Int16x8.splat(0);
  var one = SIMD.Int16x8.splat(255);
  var a = SIMD.Int16x8.select(SIMD.Int16x8.greaterThan(x, SIMD.Int16x8.fromUint8x16Bits(k00ff00ff)),one,zero);//
  var b = SIMD.Int16x8.and(x, SIMD.Int16x8.not(a));
  var b = SIMD.Int16x8.select(SIMD.Int16x8.lessThan(b, zero),zero,b);
  var c = SIMD.Int16x8.shiftRightByScalar(SIMD.Int16x8.and(x, a), 15);//fifteen
  var d = SIMD.Int16x8.and(SIMD.Int16x8.not(c), a);
  var e = SIMD.Int16x8.or(b, d);
  var final = SIMD.Uint8x16.shuffle(
      SIMD.Uint8x16.fromInt16x8Bits(e), SIMD.Uint8x16.fromInt16x8Bits(e), 0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26,
      28, 30);
  return final;
}
//------------------------------------------------------------------------------
// Transforms (Paragraph 14.4)

// Transpose two 4x4 16b matrices horizontally stored in registers.
function VP8Transpose_2_4x4_16b(
    in0, in1,//const int16x8* const const int16x8* const 
    in2, in3, out0,//const int16x8* const const int16x8* const int16x8* const 
    out1, out2, out3) {//int16x8* const int16x8* const int16x8* const 
  // Transpose the two 4x4.
  // a00 a01 a02 a03   b00 b01 b02 b03
  // a10 a11 a12 a13   b10 b11 b12 b13
  // a20 a21 a22 a23   b20 b21 b22 b23
  // a30 a31 a32 a33   b30 b31 b32 b33
  var transpose0_0 = _unpacklo_epi16(in0, in1);//const int32x4
  var transpose0_1 = _unpacklo_epi16(in2, in3);
  var transpose0_2 = _unpackhi_epi16(in0, in1);
  var transpose0_3 = _unpackhi_epi16(in2, in3);
  // a00 a10 a01 a11   a02 a12 a03 a13
  // a20 a30 a21 a31   a22 a32 a23 a33
  // b00 b10 b01 b11   b02 b12 b03 b13
  // b20 b30 b21 b31   b22 b32 b23 b33
  var transpose1_0 = _unpacklo_epi32(transpose0_0, transpose0_1);
  var transpose1_1 = _unpacklo_epi32(transpose0_2, transpose0_3);
  var transpose1_2 = _unpackhi_epi32(transpose0_0, transpose0_1);
  var transpose1_3 = _unpackhi_epi32(transpose0_2, transpose0_3);
  // a00 a10 a20 a30 a01 a11 a21 a31
  // b00 b10 b20 b30 b01 b11 b21 b31
  // a02 a12 a22 a32 a03 a13 a23 a33
  // b02 b12 a22 b32 b03 b13 b23 b33
  out0[0] = SIMD.Int16x8.fromInt32x4Bits(_unpacklo_epi64(transpose1_0, transpose1_1));
  out1[0] = SIMD.Int16x8.fromInt32x4Bits(_unpackhi_epi64(transpose1_0, transpose1_1));
  out2[0] = SIMD.Int16x8.fromInt32x4Bits(_unpacklo_epi64(transpose1_2, transpose1_3));
  out3[0] = SIMD.Int16x8.fromInt32x4Bits(_unpackhi_epi64(transpose1_2, transpose1_3));
  // a00 a10 a20 a30   b00 b10 b20 b30
  // a01 a11 a21 a31   b01 b11 b21 b31
  // a02 a12 a22 a32   b02 b12 b22 b32
  // a03 a13 a23 a33   b03 b13 b23 b33
}

function Transform(in_, in_off, dst, dst_off, do_two) {//const int16_t* uint8_t* 
  var k1 = SIMD.Int32x4(20091, 20091, 20091, 20091);
  var k2 = SIMD.Int32x4(35468, 35468, 35468, 35468);
  var T0=[], T1=[], T2=[], T3=[];//int16x8

  // Load and concatenate the transform coefficients (we'll do two transforms
  // in parallel). In the case of only one transform, the second half of the
  // vectors will just contain random value we'll never use nor store.
  var in0, in1, in2, in3;//int16x8
  {
    in0 = SIMD.Int16x8.fromUint8x16Bits(get_8_bytes_fromInt16bits(in_,in_off+ 0));
    in1 = SIMD.Int16x8.fromUint8x16Bits(get_8_bytes_fromInt16bits(in_,in_off+ 4));
    in2 = SIMD.Int16x8.fromUint8x16Bits(get_8_bytes_fromInt16bits(in_,in_off+ 8));
    in3 = SIMD.Int16x8.fromUint8x16Bits(get_8_bytes_fromInt16bits(in_,in_off+ 12));
    // a00 a10 a20 a30   x x x x
    // a01 a11 a21 a31   x x x x
    // a02 a12 a22 a32   x x x x
    // a03 a13 a23 a33   x x x x
    if (do_two) {
      var inB0 = SIMD.Int32x4.fromUint8x16Bits(get_8_bytes_fromInt16bits(in_,in_off+ 16));//const int16x8
      var inB1 = SIMD.Int32x4.fromUint8x16Bits(get_8_bytes_fromInt16bits(in_,in_off+ 20));
      var inB2 = SIMD.Int32x4.fromUint8x16Bits(get_8_bytes_fromInt16bits(in_,in_off+ 24));
      var inB3 = SIMD.Int32x4.fromUint8x16Bits(get_8_bytes_fromInt16bits(in_,in_off+ 28));
      in0 = SIMD.Int16x8.fromInt32x4Bits(_unpacklo_epi64(SIMD.Int32x4.fromInt16x8Bits(in0), inB0));
      in1 = SIMD.Int16x8.fromInt32x4Bits(_unpacklo_epi64(SIMD.Int32x4.fromInt16x8Bits(in1), inB1));
      in2 = SIMD.Int16x8.fromInt32x4Bits(_unpacklo_epi64(SIMD.Int32x4.fromInt16x8Bits(in2), inB2));
      in3 = SIMD.Int16x8.fromInt32x4Bits(_unpacklo_epi64(SIMD.Int32x4.fromInt16x8Bits(in3), inB3));
      // a00 a10 a20 a30   b00 b10 b20 b30
      // a01 a11 a21 a31   b01 b11 b21 b31
      // a02 a12 a22 a32   b02 b12 b22 b32
      // a03 a13 a23 a33   b03 b13 b23 b33
    }
  }

  // Vertical pass and subsequent transpose.
  {
    var a = SIMD.Int16x8.add(in0, in2);//const int16x8
    var b = SIMD.Int16x8.sub(in0, in2);
    var c1 = _mulhi_int16x8(in1, k2);
    var c2 = SIMD.Int16x8.add(_mulhi_int16x8(in3, k1), in3);
    var c = SIMD.Int16x8.sub(c1, c2);
    var d1 = SIMD.Int16x8.add(_mulhi_int16x8(in1, k1), in1);
    var d2 = _mulhi_int16x8(in3, k2);
    var d = SIMD.Int16x8.add(d1, d2);

    // Second pass.
    var tmp0 = SIMD.Int16x8.add(a, d);
    var tmp1 = SIMD.Int16x8.add(b, c);
    var tmp2 = SIMD.Int16x8.sub(b, c);
    var tmp3 = SIMD.Int16x8.sub(a, d);

    // Transpose the two 4x4.
    VP8Transpose_2_4x4_16b(tmp0, tmp1, tmp2, tmp3, T0, T1, T2, T3);
  }

  // Horizontal pass and subsequent transpose.
  {
    var four = SIMD.Int16x8(4, 4, 4, 4, 4, 4, 4, 4);//const int16x8
    var dc = SIMD.Int16x8.add(T0[0], four);
    var a = SIMD.Int16x8.add(dc, T2[0]);
    var b = SIMD.Int16x8.sub(dc, T2[0]);
    var c1 = _mulhi_int16x8(T1[0], k2);
    var c2 = SIMD.Int16x8.add(_mulhi_int16x8(T3[0], k1), T3[0]);
    var c = SIMD.Int16x8.sub(c1, c2);
    var d1 = SIMD.Int16x8.add(_mulhi_int16x8(T1[0], k1), T1[0]);
    var d2 = _mulhi_int16x8(T3[0], k2);
    var d = SIMD.Int16x8.add(d1, d2);

    // Second pass.
    var tmp0 = SIMD.Int16x8.add(a, d);
    var tmp1 = SIMD.Int16x8.add(b, c);
    var tmp2 = SIMD.Int16x8.sub(b, c);
    var tmp3 = SIMD.Int16x8.sub(a, d);
    var three = SIMD.Int16x8(3, 3, 3, 3, 3, 3, 3, 3);
    var shifted0 = SIMD.Int16x8.shiftRightByScalar(tmp0, 3);//three
    var shifted1 = SIMD.Int16x8.shiftRightByScalar(tmp1, 3);
    var shifted2 = SIMD.Int16x8.shiftRightByScalar(tmp2, 3);
    var shifted3 = SIMD.Int16x8.shiftRightByScalar(tmp3, 3);

    // Transpose the two 4x4.
    VP8Transpose_2_4x4_16b(shifted0, shifted1, shifted2, shifted3, T0, T1,
                           T2, T3);
  }

  // Add inverse transform to 'dst' and store.
  {
    var zero = SIMD.Int8x16.splat(0);//const int8x16
    // Load the reference(s).
    var dst0 = SIMD.Int8x16.splat(0), dst1 = SIMD.Int8x16.splat(0), dst2 = SIMD.Int8x16.splat(0), dst3 = SIMD.Int8x16.splat(0);//int16x8
    if (do_two) {
      // Load eight bytes/pixels per line.
      dst0 = SIMD.Int8x16.fromUint8x16Bits(get_8_bytes(dst,dst_off + 0 * BPS));
      dst1 = SIMD.Int8x16.fromUint8x16Bits(get_8_bytes(dst,dst_off + 1 * BPS));
      dst2 = SIMD.Int8x16.fromUint8x16Bits(get_8_bytes(dst,dst_off + 2 * BPS));
      dst3 = SIMD.Int8x16.fromUint8x16Bits(get_8_bytes(dst,dst_off + 3 * BPS));
    } else {
      // Load four bytes/pixels per line.
	  for(var i=0;i<4;++i) dst0 = SIMD.Int8x16.replaceLane(dst0,i,(dst[dst_off + 0 * BPS +i]));
      //memcpy(&dst0, (dst + 0 * BPS), 4);
	  for(var i=0;i<4;++i) dst1 = SIMD.Int8x16.replaceLane(dst1,i,(dst[dst_off + 1 * BPS +i]));
      //memcpy(&dst1, (dst + 1 * BPS), 4);
	  for(var i=0;i<4;++i) dst2 = SIMD.Int8x16.replaceLane(dst2,i,(dst[dst_off + 2 * BPS +i]));
      //memcpy(&dst2, (dst + 2 * BPS), 4);
	  for(var i=0;i<4;++i) dst3 = SIMD.Int8x16.replaceLane(dst3,i,(dst[dst_off + 3 * BPS +i]));
      //memcpy(&dst3, (dst + 3 * BPS), 4);
    }
    // Convert to 16b.
    dst0 = _unpacklo_epi8(dst0, zero);
    dst1 = _unpacklo_epi8(dst1, zero);
    dst2 = _unpacklo_epi8(dst2, zero);
    dst3 = _unpacklo_epi8(dst3, zero);
    // Add the inverse transform(s).
    dst0 = SIMD.Int16x8.add(dst0, T0[0]);
    dst1 = SIMD.Int16x8.add(dst1, T1[0]);
    dst2 = SIMD.Int16x8.add(dst2, T2[0]);
    dst3 = SIMD.Int16x8.add(dst3, T3[0]);
    // Unsigned saturate to 8b.
    dst0 = int16x8_to_uint8x16_sat(dst0);
    dst1 = int16x8_to_uint8x16_sat(dst1);
    dst2 = int16x8_to_uint8x16_sat(dst2);
    dst3 = int16x8_to_uint8x16_sat(dst3);
    // Store the results.
    if (do_two) {
      // Store eight bytes/pixels per line.
      // TODO: use lanes instead ???
      for(var i=0;i<8;++i) dst[dst_off + 0 * BPS +i] = SIMD.Uint8x16.extractLane(dst0,i);
	  //memcpy(dst + 0 * BPS, &dst0, 8);
      for(var i=0;i<8;++i) dst[dst_off + 1 * BPS +i] = SIMD.Uint8x16.extractLane(dst1,i);
      //memcpy(dst + 1 * BPS, &dst1, 8);
      for(var i=0;i<8;++i) dst[dst_off + 2 * BPS +i] = SIMD.Uint8x16.extractLane(dst2,i);
      //memcpy(dst + 2 * BPS, &dst2, 8);
      for(var i=0;i<8;++i) dst[dst_off + 3 * BPS +i] = SIMD.Uint8x16.extractLane(dst3,i);
      //memcpy(dst + 3 * BPS, &dst3, 8);
    } else {
      // Store four bytes/pixels per line.
      for(var i=0;i<4;++i) dst[dst_off + 0 * BPS +i] = SIMD.Uint8x16.extractLane(dst0,i);
	  //memcpy(dst + 0 * BPS, &dst0, 4);
      for(var i=0;i<4;++i) dst[dst_off + 1 * BPS +i] = SIMD.Uint8x16.extractLane(dst1,i);
      //memcpy(dst + 1 * BPS, &dst1, 4);
      for(var i=0;i<4;++i) dst[dst_off + 2 * BPS +i] = SIMD.Uint8x16.extractLane(dst2,i);
      //memcpy(dst + 2 * BPS, &dst2, 4);
      for(var i=0;i<4;++i) dst[dst_off + 3 * BPS +i] = SIMD.Uint8x16.extractLane(dst3,i);
      //memcpy(dst + 3 * BPS, &dst3, 4);
    }
  }
}

//------------------------------------------------------------------------------
// 4x4 predictions

function SIMD_DST(dst,dst_off,x, y, z) { dst[dst_off+(x) + (y) * BPS]=z; }
function SIMD_AVG2(a, b) { return (((a) + (b) + 1) >> 1) }
function SIMD_AVG3(a, b, c) { return ((((a) + 2 * (b) + (c) + 2) >> 2)) } //(uint8_t)

function VE4(dst,dst_off) {//uint8_t*   // vertical
  var zero = SIMD.Uint8x16.splat(0);
  var two = SIMD.Uint16x8(2, 2, 2, 2, 2, 2, 2, 2);
  var top = get_8_bytes(dst,dst_off - BPS - 1);//const uint8x16
  var ABCDEFGH = SIMD.Uint8x16.shuffle(
      top, zero, 0, 16, 1, 16, 2, 16, 3, 16, 4, 16, 5, 16, 6, 16, 7, 16);
  var BCDEFGHX = SIMD.Uint8x16.shuffle(
      top, zero, 1, 16, 2, 16, 3, 16, 4, 16, 5, 16, 6, 16, 7, 16, 16, 16);
  var CDEFGHXX = SIMD.Uint8x16.shuffle(
      top, zero, 2, 16, 3, 16, 4, 16, 5, 16, 6, 16, 7, 16, 16, 16, 16, 16);
	  var sum0=SIMD.Uint16x8.add(SIMD.Uint16x8.fromUint8x16Bits(ABCDEFGH), SIMD.Uint16x8.fromUint8x16Bits(BCDEFGHX));
	  var sum1=SIMD.Uint16x8.add(SIMD.Uint16x8.fromUint8x16Bits(BCDEFGHX), SIMD.Uint16x8.fromUint8x16Bits(CDEFGHXX));
	  var sum2=SIMD.Uint16x8.add(sum1, two);
	  var sum=SIMD.Uint16x8.add(sum0,sum2);
  var avg3 = SIMD.Uint8x16.fromUint16x8Bits(SIMD.Uint16x8.shiftRightByScalar(
      sum, 2));//const uint16x8 two
  var vals = SIMD.Uint32x4.fromUint8x16Bits(SIMD.Uint8x16.shuffle(
      avg3, zero, 0, 2, 4, 6, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16,
      16, 16));
  var i=0;//int
  for (i = 0; i < 4; ++i) {
    WebPUint32ToMem(dst,dst_off + i * BPS, SIMD.Uint32x4.extractLane(vals,0));
  }
}

function RD4(dst,dst_off) {//uint8_t*   // Down-right
  var zero = SIMD.Uint8x16.splat(0);
  var two = SIMD.Uint16x8(2, 2, 2, 2, 2, 2, 2, 2);
  var top = get_8_bytes(dst,dst_off - BPS - 1);//const uint8x16
  var I = dst[dst_off-1 + 0 * BPS];
  var J = dst[dst_off-1 + 1 * BPS];
  var K = dst[dst_off-1 + 2 * BPS];
  var L = dst[dst_off-1 + 3 * BPS];
  var LKJI_____ =
      SIMD.Uint8x16.fromUint32x4Bits(cvt32_to_128((L | (K << 8) | (J << 16) | (I << 24))>>>0));
  var lkjixabcd = SIMD.Uint8x16.shuffle(
      LKJI_____, top, 0, 1, 2, 3, 16, 17, 18, 19, 20, 31, 31, 31, 31,
      31, 31, 31);
  var LKJIXABC = SIMD.Uint8x16.shuffle(
      lkjixabcd, zero, 0, 16, 1, 16, 2, 16, 3, 16, 4, 16, 5, 16, 6, 16, 7, 16);
  var KJIXABCD_ = SIMD.Uint8x16.shuffle(
      lkjixabcd, zero, 1, 16, 2, 16, 3, 16, 4, 16, 5, 16, 6, 16, 7, 16, 8, 16);
  var JIXABCD__ = SIMD.Uint8x16.shuffle(
      lkjixabcd, zero, 2, 16, 3, 16, 4, 16, 5, 16, 6, 16, 7, 16, 8, 16, 9, 16);
	  var sum0=SIMD.Uint16x8.add(SIMD.Uint16x8.fromUint8x16Bits(LKJIXABC), SIMD.Uint16x8.fromUint8x16Bits(KJIXABCD_));
	  var sum1=SIMD.Uint16x8.add(SIMD.Uint16x8.fromUint8x16Bits(KJIXABCD_), SIMD.Uint16x8.fromUint8x16Bits(JIXABCD__));
	  var sum2=SIMD.Uint16x8.add(sum1, two);
	  var sum=SIMD.Uint16x8.add(sum0,sum2);
  var avg3 = SIMD.Uint8x16.fromUint16x8Bits(SIMD.Uint16x8.shiftRightByScalar(
      sum, 2));//const uint16x8 two
  var vals0 = SIMD.Uint32x4.fromUint8x16Bits(SIMD.Uint8x16.shuffle(
      avg3, zero, 6, 8, 10, 12, 16, 16, 16, 16, 16, 16, 16, 16, 16,
      16, 16, 16));
  var vals1 = SIMD.Uint32x4.fromUint8x16Bits(SIMD.Uint8x16.shuffle(
      avg3, zero, 4, 6, 8, 10, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16,
      16, 16));
  var vals2 = SIMD.Uint32x4.fromUint8x16Bits(SIMD.Uint8x16.shuffle(
      avg3, zero, 2, 4, 6, 8, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16,
      16, 16));
  var vals3 = SIMD.Uint32x4.fromUint8x16Bits(SIMD.Uint8x16.shuffle(
      avg3, zero, 0, 2, 4, 6, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16,
      16, 16));
  WebPUint32ToMem(dst,dst_off + 0 * BPS, SIMD.Uint32x4.extractLane(vals0,0));
  WebPUint32ToMem(dst,dst_off + 1 * BPS, SIMD.Uint32x4.extractLane(vals1,0));
  WebPUint32ToMem(dst,dst_off + 2 * BPS, SIMD.Uint32x4.extractLane(vals2,0));
  WebPUint32ToMem(dst,dst_off + 3 * BPS, SIMD.Uint32x4.extractLane(vals3,0));
}

function VR4(dst,dst_off) {//uint8_t*   // Vertical-Right
  var I = dst[dst_off-1 + 0 * BPS];//const uint32_t
  var J = dst[dst_off-1 + 1 * BPS];
  var K = dst[dst_off-1 + 2 * BPS];
  var X = dst[dst_off-1 - BPS];
  var zero = SIMD.Uint8x16.splat(0);
  var one = SIMD.Uint16x8(1, 1, 1, 1, 1, 1, 1, 1);
  var two = SIMD.Uint16x8(2, 2, 2, 2, 2, 2, 2, 2);
  var top = get_8_bytes(dst,dst_off - BPS - 1);//const uint8x16
  var XABCD = SIMD.Uint8x16.shuffle(
      top, zero, 0, 16, 1, 16, 2, 16, 3, 16, 4, 16, 16, 16, 16, 16, 16, 16);
  var ABCD0 = SIMD.Uint8x16.shuffle(
      top, zero, 1, 16, 2, 16, 3, 16, 4, 16, 16, 16, 16, 16, 16, 16, 16, 16);
  var abcd = SIMD.Uint8x16.fromUint16x8Bits(SIMD.Uint16x8.shiftRightByScalar(SIMD.Uint16x8.add(SIMD.Uint16x8.fromUint8x16Bits(XABCD), SIMD.Uint16x8.add(SIMD.Uint16x8.fromUint8x16Bits(ABCD0), one)), 1));//const uint16x8 one
  var IX = SIMD.Uint8x16.fromUint32x4Bits(cvt32_to_128(I | (X << 8)));//const uint16x8 (uint16x8)
  var IXABCD = SIMD.Uint8x16.shuffle(
      XABCD, IX, 16, 31, 17, 31, 2, 3, 4, 5, 6, 7, 8, 9, 10,
      11, 12, 13);
	  var sum0=SIMD.Uint16x8.add(SIMD.Uint16x8.fromUint8x16Bits(IXABCD), SIMD.Uint16x8.fromUint8x16Bits(XABCD));
	  var sum1=SIMD.Uint16x8.add(SIMD.Uint16x8.fromUint8x16Bits(XABCD), SIMD.Uint16x8.fromUint8x16Bits(ABCD0));
	  var sum2=SIMD.Uint16x8.add(sum1, two);
	  var sum=SIMD.Uint16x8.add(sum0,sum2);
  var efgh = SIMD.Uint8x16.fromUint16x8Bits(SIMD.Uint16x8.shiftRightByScalar(
      sum, 2));//const uint16x8 two
  var vals0 = SIMD.Uint32x4.fromUint8x16Bits(SIMD.Uint8x16.shuffle(
      abcd, zero, 0, 2, 4, 6, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16,
      16, 16));
  var vals1 = SIMD.Uint32x4.fromUint8x16Bits(SIMD.Uint8x16.shuffle(
      efgh, zero, 0, 2, 4, 6, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16,
      16, 16));
  var vals2 = SIMD.Uint32x4.fromUint8x16Bits(SIMD.Uint8x16.shuffle(
      abcd, zero, 16, 0, 2, 4, 6, 16, 16, 16, 16, 16, 16, 16, 16, 16,
      16, 16));
  var vals3 = SIMD.Uint32x4.fromUint8x16Bits(SIMD.Uint8x16.shuffle(
      efgh, zero, 16, 0, 2, 4, 6, 16, 16, 16, 16, 16, 16, 16, 16, 16,
      16, 16));
  WebPUint32ToMem(dst,dst_off + 0 * BPS, SIMD.Uint32x4.extractLane(vals0,0));
  WebPUint32ToMem(dst,dst_off + 1 * BPS, SIMD.Uint32x4.extractLane(vals1,0));
  WebPUint32ToMem(dst,dst_off + 2 * BPS, SIMD.Uint32x4.extractLane(vals2,0));
  WebPUint32ToMem(dst,dst_off + 3 * BPS, SIMD.Uint32x4.extractLane(vals3,0));

  SIMD_DST(dst,dst_off,0, 2, SIMD_AVG3(J, I, X));
  SIMD_DST(dst,dst_off,0, 3, SIMD_AVG3(K, J, I));
}

function LD4(dst,dst_off) {//uint8_t*   // Down-Left
  var zero = SIMD.Uint8x16.splat(0);
  var two = SIMD.Uint16x8(2, 2, 2, 2, 2, 2, 2, 2);
  var top = get_8_bytes(dst,dst_off - BPS);//const uint8x16
  var ABCDEFGH = SIMD.Uint8x16.shuffle(
      top, zero, 0, 16, 1, 16, 2, 16, 3, 16, 4, 16, 5, 16, 6, 16, 7, 16);
  var BCDEFGH0 = SIMD.Uint8x16.shuffle(
      top, zero, 1, 16, 2, 16, 3, 16, 4, 16, 5, 16, 6, 16, 7, 16, 16, 16);
  var CDEFGHH0 = SIMD.Uint8x16.shuffle(
      top, zero, 2, 16, 3, 16, 4, 16, 5, 16, 6, 16, 7, 16, 7, 16, 16, 16);
	  var sum0=SIMD.Uint16x8.add(SIMD.Uint16x8.fromUint8x16Bits(ABCDEFGH), SIMD.Uint16x8.fromUint8x16Bits(BCDEFGH0));
	  var sum1=SIMD.Uint16x8.add(SIMD.Uint16x8.fromUint8x16Bits(BCDEFGH0), SIMD.Uint16x8.fromUint8x16Bits(CDEFGHH0));
	  var sum2=SIMD.Uint16x8.add(sum1, two);
	  var sum=SIMD.Uint16x8.add(sum0,sum2);
  var avg3 = SIMD.Uint8x16.fromUint16x8Bits(SIMD.Uint16x8.shiftRightByScalar(
      sum, 2));//const uint16x8 two
  var vals0 = SIMD.Uint32x4.fromUint8x16Bits(SIMD.Uint8x16.shuffle(
      avg3, zero, 0, 2, 4, 6, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16,
      16, 16));
  var vals1 = SIMD.Uint32x4.fromUint8x16Bits(SIMD.Uint8x16.shuffle(
      avg3, zero, 2, 4, 6, 8, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16,
      16, 16));
  var vals2 = SIMD.Uint32x4.fromUint8x16Bits(SIMD.Uint8x16.shuffle(
      avg3, zero, 4, 6, 8, 10, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16,
      16, 16));
  var vals3 = SIMD.Uint32x4.fromUint8x16Bits(SIMD.Uint8x16.shuffle(
      avg3, zero, 6, 8, 10, 12, 16, 16, 16, 16, 16, 16, 16, 16, 16,
      16, 16, 16));
  WebPUint32ToMem(dst,dst_off + 0 * BPS, SIMD.Uint32x4.extractLane(vals0,0));
  WebPUint32ToMem(dst,dst_off + 1 * BPS, SIMD.Uint32x4.extractLane(vals1,0));
  WebPUint32ToMem(dst,dst_off + 2 * BPS, SIMD.Uint32x4.extractLane(vals2,0));
  WebPUint32ToMem(dst,dst_off + 3 * BPS, SIMD.Uint32x4.extractLane(vals3,0));
}

function VL4(dst,dst_off) {//uint8_t*   // Vertical-Left
  var zero = SIMD.Uint8x16.splat(0);
  var one = SIMD.Uint16x8(1, 1, 1, 1, 1, 1, 1, 1);
  var two = SIMD.Uint16x8(2, 2, 2, 2, 2, 2, 2, 2);
  var top = get_8_bytes(dst,dst_off - BPS);//const uint8x16
  var ABCDEFGH = SIMD.Uint8x16.shuffle(
      top, zero, 0, 16, 1, 16, 2, 16, 3, 16, 4, 16, 5, 16, 6, 16, 7, 16);
  var BCDEFGH_ = SIMD.Uint8x16.shuffle(
      top, zero, 1, 16, 2, 16, 3, 16, 4, 16, 5, 16, 6, 16, 7, 16, 16, 16);
  var CDEFGH__ = SIMD.Uint8x16.shuffle(
      top, zero, 2, 16, 3, 16, 4, 16, 5, 16, 6, 16, 7, 16, 16, 16, 16, 16);
  var avg1 = SIMD.Uint8x16.fromUint16x8Bits(SIMD.Uint16x8.shiftRightByScalar(SIMD.Uint16x8.add(SIMD.Uint16x8.fromUint8x16Bits(ABCDEFGH), SIMD.Uint16x8.add(SIMD.Uint16x8.fromUint8x16Bits(BCDEFGH_), one)), 1));//const uint16x8
	  var sum0=SIMD.Uint16x8.add(SIMD.Uint16x8.fromUint8x16Bits(ABCDEFGH), SIMD.Uint16x8.fromUint8x16Bits(BCDEFGH_));
	  var sum1=SIMD.Uint16x8.add(SIMD.Uint16x8.fromUint8x16Bits(BCDEFGH_), SIMD.Uint16x8.fromUint8x16Bits(CDEFGH__));
	  var sum2=SIMD.Uint16x8.add(sum1, two);
	  var sum=SIMD.Uint16x8.add(sum0,sum2);
  var avg3 = SIMD.Uint8x16.fromUint16x8Bits(SIMD.Uint16x8.shiftRightByScalar(
      sum, 2));//const uint16x8 two
  var vals0 = SIMD.Uint32x4.fromUint8x16Bits(SIMD.Uint8x16.shuffle(
      avg1, zero, 0, 2, 4, 6, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16,
      16, 16));
  var vals1 = SIMD.Uint32x4.fromUint8x16Bits(SIMD.Uint8x16.shuffle(
      avg3, zero, 0, 2, 4, 6, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16,
      16, 16));
  var vals2 = SIMD.Uint32x4.fromUint8x16Bits(SIMD.Uint8x16.shuffle(
      avg1, zero, 2, 4, 6, 8, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16,
      16, 16));
  var vals3 = SIMD.Uint32x4.fromUint8x16Bits(SIMD.Uint8x16.shuffle(
      avg3, zero, 2, 4, 6, 8, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16,
      16, 16));
  var vals4 = SIMD.Uint32x4.fromUint8x16Bits(SIMD.Uint8x16.shuffle(
      avg3, zero, 8, 10, 12, 14, 16, 16, 16, 16, 16, 16, 16, 16, 16,
      16, 16, 16));
  var extra_out = SIMD.Uint32x4.extractLane(vals4,0);//const uint32_t
  WebPUint32ToMem(dst,dst_off + 0 * BPS, SIMD.Uint32x4.extractLane(vals0,0));
  WebPUint32ToMem(dst,dst_off + 1 * BPS, SIMD.Uint32x4.extractLane(vals1,0));
  WebPUint32ToMem(dst,dst_off + 2 * BPS, SIMD.Uint32x4.extractLane(vals2,0));
  WebPUint32ToMem(dst,dst_off + 3 * BPS, SIMD.Uint32x4.extractLane(vals3,0));

  SIMD_DST(dst,dst_off,3, 2, (extra_out >> 0) & 0xff);
  SIMD_DST(dst,dst_off,3, 3, (extra_out >> 8) & 0xff);
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
  VP8Transform = Transform;

  VP8PredLuma4[2] = VE4;
  VP8PredLuma4[4] = RD4;
  VP8PredLuma4[5] = VR4;
  VP8PredLuma4[6] = LD4;
  VP8PredLuma4[7] = VL4;

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
