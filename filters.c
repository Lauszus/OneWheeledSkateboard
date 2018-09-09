/* Copyright (C) 2018 Kristian Sloth Lauszus. All rights reserved.

 This software may be distributed and modified under the terms of the GNU
 General Public License version 2 (GPL2) as published by the Free Software
 Foundation and appearing in the file GPL2.TXT included in the packaging of
 this file. Please note that GPL2 Section 2[b] requires that all works based
 on this software must also be made publicly available under the terms of
 the GPL2 ("Copyleft").

 Contact information
 -------------------

 Kristian Sloth Lauszus
 Web      :  http://www.lauszus.com
 e-mail   :  lauszus@gmail.com
*/

#include <Arduino.h>

#include "filters.h"

float FILTERS_ApplyLowPass(low_pass_t *low_pass, float input, float dt) {
    const float tau = 1.0f/(TWO_PI*low_pass->Fc); // Time constant
    const float alpha = dt/(tau + dt); // See (10) in http://techteach.no/simview/lowpass_filter/doc/filter_algorithm.pdf

    // y(n) = y(n-1) + alpha*(u(n) - y(n-1))
    float output = low_pass->prevOutput + alpha*(input - low_pass->prevOutput);
    low_pass->prevOutput = output;
    return output;
}

float FILTERS_ApplyHighPass(high_pass_t *high_pass, float input, float dt) {
    const float tau = 1.0f/(TWO_PI*high_pass->Fc); // Time constant
    const float alpha = tau/(tau + dt);

    // y(n) = alpha*y(n-1) + alpha*(u(n) - u(n-1))
    float output = alpha*high_pass->prevOutput + alpha*(input - high_pass->prevInput);
    high_pass->prevOutput = output;
    high_pass->prevInput = input;
    return output;
}
