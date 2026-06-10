#include "cquik.h"

#define CQUIK_IMPLEMENTATION
#include "cquik.h"

int main(void)
{
    double identity[16];

    cquik_mat4_identity(identity);
    return identity[0] == 1.0 && identity[5] == 1.0 &&
        identity[10] == 1.0 && identity[15] == 1.0 ? 0 : 1;
}
