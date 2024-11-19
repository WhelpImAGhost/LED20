#include <stdio.h>
#include <stdlib.h>
#include <math.h>



// Helper function to compare values for qsort
int compare(const void *a, const void *b) {
    float diff = *(float *)a - *(float *)b;
    return (diff > 0) - (diff < 0);
}

// Function to compute the mode with tolerance
float mode_with_tolerance(float *array, size_t size, float tolerance) {
    if (size == 0) return NAN; // Handle empty array

    // Sort the array
    qsort(array, size, sizeof(float), compare);

    // Variables to track the mode
    float mode = array[0];
    int max_count = 1;

    // Variables to track the current group
    int current_count = 1;
    float group_start = array[0];

    for (size_t i = 1; i < size; i++) {
        // Check if the value is within the tolerance range of the group start
        if (fabs(array[i] - group_start) <= fabs(group_start * tolerance)) {
            current_count++; // Increment the count for the current group
        } else {
            // If outside tolerance, start a new group
            if (current_count > max_count) {
                max_count = current_count;
                mode = group_start;
            }
            group_start = array[i];
            current_count = 1;
        }
    }

    // Final group check
    if (current_count > max_count) {
        mode = group_start;
    }

    return mode;
}