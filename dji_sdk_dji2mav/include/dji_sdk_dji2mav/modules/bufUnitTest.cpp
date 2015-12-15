#include <iostream>
#include <stdint.h>
#include "moduleBuf.h"
#include <string>

int main(int argc, char *argv[]) {
    uint8_t *str = new uint8_t[100];
    char *strs = "0123456789abcdefg";
    for(int i=0;i<20;++i) str[i] = strs[i];
    uint8_t *re = new uint8_t[100];
    memset(re, 0, sizeof(uint8_t) * 100);

    dji2mav::ModuleBuf buf1(0);
    std::cout << "buf1: " << buf1.writeBuf(str, 1) << ", " << buf1.readBuf(re, 1) << std::endl;
    //false, false

    dji2mav::ModuleBuf buf2(1);
    std::cout << "buf2: " << buf2.writeBuf(str + 3, 1) << ", " << buf2.readBuf(re, 1) << ", " << buf2.writeBuf(str + 2, 1) << ", " << buf2.writeBuf(str + 1, 1) << ", " << buf2.readBuf(re + 1, 1) << std::endl;
    std::cout << re[0] << re[1] << std::endl;
    //true, true, true, false, true
    //32

    dji2mav::ModuleBuf buf3(2);
    std::cout << "buf3: " << buf3.readBuf(re+5, 0) << ", " << buf3.readBuf(re+6, 1) << ", " << buf3.writeBuf(str + 5, 2) << ", " << buf3.readBuf(re+7, 1) << ", " << buf3.readBuf(re+8, 1) << ", " << buf3.writeBuf(str+10, 2) << ", " << buf3.readBuf(re+9, 2) << std::endl;
    std::cout << (char)re[5] << (char)re[6] << (char)re[7] << (char)re[8] << (char)re[9] << (char)re[10] << std::endl;
    //true, false, true, true, true, true, true
    //0056ab

    dji2mav::ModuleBuf buf4(10);
    std:: cout << "buf4: " << buf4.writeBuf(str, 5) << ", " << buf4.readBuf(re+10, 4) << ", " << buf4.writeBuf(str+5, 3) << ", " << buf4.readBuf(re+14, 4) << std::endl;
    std::cout << re[10] << re[11] << re[12] << re[13] << re[14] << re[15] << re[16] << re[17] << std::endl;

    return 0;
}
