#include <stdint.h>

#include "Board.h"

static AT91PS_PWMC pwm = AT91C_BASE_PWMC;

/* find highest bit set. returns bit (32..1) or 0 in case no bit set  */
static int fhs(uint32_t val)
{
    int i;
    for (i = 32; i > 0; i--)
    {
        if (val & (1 << (i-1)))
            return i;
    }
    return 0;
}

/* set frequency of PWM signal to freq */
int pwmFreqSet(int channel, uint32_t freq)
{
    /* in order to get maximum resolution, the pre-scaler must be set to
     * something like freq << 16.  However, the mimimum pre-scaled frequency
     * we can get is MCLK (48MHz), the minimum is MCLK/(1024*255) =
     * 48MHz/261120 = 183Hz */
    uint32_t overall_div;
    uint32_t presc_total;
    uint8_t cpre = 0;
    uint16_t cprd;

    if (freq > MCK)
        return -1;
    overall_div = MCK / freq;

    if (overall_div > 0x7fff)
    {
        // divisor is larger than half the maximum CPRD register, we
        // have to configure prescalers
        presc_total = overall_div >> 15;
        // find highest 2^n fitting in prescaler (highest bit set)
        cpre = fhs(presc_total);
        if (cpre > 0)
        {
            // subtract one, because of fhs semantics //
            cpre--;
        }
        cprd = overall_div / (1 << cpre);
    }
    else
        cprd = overall_div;

    AT91F_PWMC_CfgChannel(AT91C_BASE_PWMC, channel, cpre|AT91C_PWMC_CPOL, cprd, 2);

    return 0;
}

/* set frequency of PWM signal to freq */
int pwmFreqSetInv(int channel, uint32_t freq)
{
    /* in order to get maximum resolution, the pre-scaler must be set to
     * something like freq << 16.  However, the mimimum pre-scaled frequency
     * we can get is MCLK (48MHz), the minimum is MCLK/(1024*255) =
     * 48MHz/261120 = 183Hz */
    uint32_t overall_div;
    uint32_t presc_total;
    uint8_t cpre = 0;
    uint16_t cprd;

    if (freq > MCK)
        return -1;
    overall_div = MCK / freq;

    if (overall_div > 0x7fff)
    {
        // divisor is larger than half the maximum CPRD register, we
        // have to configure prescalers
        presc_total = overall_div >> 15;
        // find highest 2^n fitting in prescaler (highest bit set)
        cpre = fhs(presc_total);
        if (cpre > 0)
        {
            // subtract one, because of fhs semantics //
            cpre--;
        }
        cprd = overall_div / (1 << cpre);
    }
    else
        cprd = overall_div;

    AT91F_PWMC_CfgChannel(AT91C_BASE_PWMC, channel, cpre, cprd, 2);

    return 0;
}

void pwmDutySetPercent(int channel, uint16_t duty)
{
    uint32_t tmp = pwm->PWMC_CH[channel].PWMC_CPRDR & 0xffff;

    tmp = tmp << 16;    /* extend value by 2^16 */
    tmp = tmp / 100;    /* tmp = 1 % of extended cprd */
    tmp = duty * tmp;   /* tmp = 'duty' % of extended cprd */
    tmp = tmp >> 16;    /* un-extend tmp (divide by 2^16) */

    AT91F_PWMC_UpdateChannel(AT91C_BASE_PWMC, channel, tmp);
}

void pwmDutySet_u8(int channel, uint16_t duty)
{
    //while (pwm->PWMC_CH[channel].PWMC_CCNTR<3);
    //if (duty<2) duty = 2;
    uint32_t tmp = pwm->PWMC_CH[channel].PWMC_CPRDR & 0xffff;

    tmp = tmp << 16;    /* extend value by 2^16 */
    tmp = tmp / 256;    /* tmp = 1 % of extended cprd */
    tmp = duty * tmp;   /* tmp = 'duty' % of extended cprd */
    tmp = tmp >> 16;    /* un-extend tmp (divide by 2^16) */
    AT91F_PWMC_UpdateChannel(AT91C_BASE_PWMC, channel, tmp);
}

