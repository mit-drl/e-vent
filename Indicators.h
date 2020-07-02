/**
 * MIT Emergency Ventilator Controller
 * 
 * MIT License:
 * 
 * Copyright (c) 2020 MIT
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * Dot Matrix Liquid Crystal Display Controller/Driver
 * HD44780U (LCD-II): 
 * https://www.sparkfun.com/datasheets/LCD/HD44780.pdf
 **/

#ifndef Indicators_h
#define Indicators_h

// Define Indicator icons
static const byte patientIcon[8] = {
    B01110,
    B01110,
    B00100,
    B11111,
    B11111,
    B11111,
    B01010,
    B01010};

static const byte timeIcon[8] = {
    B00000,
    B11111,
    B11111,
    B01110,
    B00100,
    B01110,
    B11111,
    B11111};

static const byte peakIcon[8] = {
    B01111,
    B10000,
    B10000,
    B00000,
    B00000,
    B00000,
    B00000,
    B00000};

static const byte plateauIcon[8] = {
    B00000,
    B10000,
    B10000,
    B01110,
    B00001,
    B00001,
    B00000,
    B00000};

static const byte peepIcon[8] = {
    B00000,
    B00000,
    B00000,
    B00000,
    B00000,
    B10000,
    B10000,
    B01111};

static const byte ieiIcon[8] = {
    B11100,
    B01011,
    B01011,
    B01000,
    B01011,
    B01011,
    B11100,
    B00000};

static const byte ie1Icon[8] = {
    B01000,
    B11011,
    B01011,
    B01000,
    B01011,
    B01011,
    B11100,
    B00000};

static const byte bellIcon[8] = {
    B00000,
    B00100,
    B01110,
    B01110,
    B01110,
    B11111,
    B00100,
    B00000};

    #endif
