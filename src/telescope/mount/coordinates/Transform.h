//--------------------------------------------------------------------------------------------------
// coordinate transformation
#pragma once

#include "../../../Common.h"

#ifdef MOUNT_PRESENT

#include <Arduino.h>
#include "../site/Site.h"
#include "Align.ref.h"
#include "Align.hs.h"

// MOTOR      <--> apply index offset and backlash        <--> INSTRUMENT  (Axis)
// INSTRUMENT <--> apply celestial coordinate conventions <--> MOUNT       (Transform)
// MOUNT      <--> apply pointing model                   <--> OBSERVED    (Transform)
// OBSERVED   <--> apply refraction                       <--> TOPOCENTRIC (Transform)

class Transform {
  public:
    // setup for coordinate transformation
    void init();

    #if DEBUG != OFF
      // prints a coordinate to the debug serial port
      void print(Coordinate *coord);
    #endif

    // converts from Mount (equatorial or horizon) to Native (equatorial and optionally horizon also) coordinates
    // returns equatorial RA
    Coordinate mountToNative(Coordinate *coord, bool returnHorizonCoords = false);
    // converts from Native (equatorial) to Mount (equatorial or horizon) coordinates (optional: a1 is h or z, a2 is d or a)
    // accepts equatorial RA
    void nativeToMount(Coordinate *coord, double *a1 = NULL, double *a2 = NULL);

    // converts from Mount to Topocentric coordinates (removes pointing model and refraction from equatorial coordinates)
    void mountToTopocentric(Coordinate *coord);
    // converts from Topocentric to Mount coordinates (adds pointing model and refraction to equatorial coordinates)
    void topocentricToMount(Coordinate *coord);

    // converts from Mount to Observed coordinates (removes pointing model from coordinates)
    void mountToObservedPlace(Coordinate *coord);
    // converts from Observed to Mount coordinates (adds pointing model to coordinates)
    void observedPlaceToMount(Coordinate *coord);

    // converts from Instrument (angular) to Mount (equatorial or horizon) coordinates
    Coordinate instrumentToMount(double a1, double a2);
    // converts from Mount (equatorial or horizon) to Instrument (angular) coordinates
    void mountToInstrument(Coordinate *coord, double *a1, double *a2);

    // converts from Topocentric to Observed coordinates (removes refraction effects from equatorial coordinates)
    void topocentricToObservedPlace(Coordinate *coord);
    // converts from Observed to Topocentric coordinates (adds refraction effects to equatorial coordinates)
    void observedPlaceToTopocentric(Coordinate *coord);

    // converts from Hour Angle (h) to Right Ascension (r) coordinates
    void hourAngleToRightAscension(Coordinate *coord, bool native);
    // converts from Right Ascension (r) to Hour Angle (h) coordinates
    // set RA to NAN to use the Hour Angle coordinate as (h)
    void rightAscensionToHourAngle(Coordinate *coord, bool native);

    // converts from Equatorial (h,d) to Horizon (a,z) coordinates
    void equToHor(Coordinate *coord);
    // converts from Equatorial (h,d) to Horizon (a) altitude coordinate
    void equToAlt(Coordinate *coord);
    // converts from Equatorial (h,d) to Horizon (a,z) coordinates
    void horToEqu(Coordinate *coord);

    // refraction at altitude, pressure (millibars), and temperature (celsius)
    // returns amount of refraction at the true altitude
    double trueRefrac(double altitude);
    // refraction at altitude, pressure (millibars), and temperature (celsius)
    // returns the amount of refraction at the apparent altitude
    double apparentRefrac(double altitude);

    #if ALIGN_MAX_NUM_STARS > 1  
      GeoAlign align;
    #endif
    int8_t mountType = MOUNT_SUBTYPE;
    bool meridianFlips;
 
  private:

    float cotf(float n);
    
    // adjust coordinate back into 0 to 360 "degrees" range (in radians)
    double backInRads(double angle);
    // adjust coordinate back into -180 to 180 "degrees" range (in radians)
    double backInRads2(double angle);
};

extern Transform transform;

#endif
