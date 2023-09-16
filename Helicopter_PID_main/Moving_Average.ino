// This is used to determine the next free spot in the profile array (MAdataStuctArray)
int numberOfProfiles = 0;

// An array that holds all the moving average profiles.
movingAverageData MAdataStuctArray[MAX_NUMBER_MA_PROFILES];

/*
   This function checks if the an profile for the analog pin with a specific value
   already exists in the MAdataStuctArray array or not.

   Return: The pointer of the existing profile if it exists.
           NULL is the profile doesn't exist.
*/
movingAverageData* checkIfMovingAverageProfileExists(int* val, int window)
{
  for (int i = 0; i < MAX_NUMBER_MA_PROFILES; i++)
  {
    if (MAdataStuctArray[i].val == val && MAdataStuctArray[i].window == window)
    {
      // Return the pointer of the profile
      return &MAdataStuctArray[i];
    }
  }
  // The profile wasn't found.
  return NULL;
}

/*
   Compute the moving average for a given analog pin and an specifick window.

   Return: The moving average value.
*/
int movingAverage(int* val, const int window)
{
  // Check if the moving average with this window size for the analog pin already exists.
  movingAverageData* MAProfile = checkIfMovingAverageProfileExists(val, window);

  // If the profile doesn't exist make one.
  if (MAProfile == NULL)
  {
    // Make a new movingAverageData profile on the next free index in the profile array.
    MAdataStuctArray[numberOfProfiles] = {val, window, 0, 0, (int*)calloc(window, sizeof(int))};

    // Assign the previous declared movingAverageData pointer to the new profile.
    MAProfile = &MAdataStuctArray[numberOfProfiles];

    // Increade the counter for the amount of profiles.
    // This counter is also used to determine the next free spot if the profile array.
    numberOfProfiles++;
  }

  // Subtract the oldest measurement value from the sum.
  MAProfile->sum -= MAProfile->valueArrayPointer[0];

  // Shift all values from the array one the left.
  // This will overwrite the first (thus oldest) measurment value
  // and frees the a spot at the end of the array to insert a new measurment value.
  for (int i = 0; i < window; i++)
  {
    MAProfile->valueArrayPointer[i] = MAProfile->valueArrayPointer[i + 1];
  }

  // Assing a new measurment value to the previous freed spot in the measurment array.
  MAProfile->valueArrayPointer[window - 1] = *val;

  // Increase the index counter by 1. This is used see how many values are added to the array in total.
  MAProfile->index++;

  // Add the newest measurment to the sum.
  MAProfile->sum += MAProfile->valueArrayPointer[window - 1];

  // If the index is greater or equal to the MA window it means that all measurment values in de array
  // are valid and thus valid averages can be calculated.
  if (MAProfile->index >= window)
  {
    // return the average of the measurment array.
    return MAProfile->sum / window;
  }
  else
  {
    // Return 0 if not all values of the measurment array are filled with valid measurment values.
    return 0;
  }
}

