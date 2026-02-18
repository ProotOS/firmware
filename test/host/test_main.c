#include <stdio.h>
#include <stdlib.h>

// Declared in each test file
int test_hid_parser_run(void);
int test_hid_event_run(void);

int main(void)
{
    int failures = 0;

    printf("========================================\n");
    printf("  Prootos Host Tests\n");
    printf("========================================\n\n");

    failures += test_hid_parser_run();
    failures += test_hid_event_run();

    printf("\n========================================\n");
    if (failures == 0) {
        printf("  ALL TESTS PASSED\n");
    } else {
        printf("  %d TEST(S) FAILED\n", failures);
    }
    printf("========================================\n");

    return failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
