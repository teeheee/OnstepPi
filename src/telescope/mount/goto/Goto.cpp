//--------------------------------------------------------------------------------------------------
// telescope mount control, sync and goto

#include "Goto.h"

#if defined(MOUNT_PRESENT)

#include "../../../lib/tasks/OnTask.h"

#include "../../Telescope.h"
#include "../Mount.h"
#include "../coordinates/Transform.h"
#include "../guide/Guide.h"
#include "../home/Home.h"
#include "../park/Park.h"
#include "../limits/Limits.h"
#include "../status/Status.h"

#if GOTO_FEATURE == ON
inline void gotoWrapper() { goTo.poll(); }
#endif

void Goto::init() {
  // confirm the data structure size
  if (GotoSettingsSize < sizeof(GotoSettings)) { nv.initError = true; DLF("ERR: Goto::init(), GotoSettingsSize error"); }

  // write the default settings to NV
  if (!nv.hasValidKey() || nv.isNull(NV_MOUNT_GOTO_BASE, sizeof(GotoSettings))) {
    VLF("MSG: Mount, goto writing defaults to NV");
    nv.writeBytes(NV_MOUNT_GOTO_BASE, &settings, sizeof(GotoSettings));
  }

  // read the settings
  nv.readBytes(NV_MOUNT_GOTO_BASE, &settings, sizeof(GotoSettings));

  // force defaults if needed
  #if MFLIP_PAUSE_HOME_MEMORY != ON
    settings.meridianFlipPause = (MFLIP_PAUSE_HOME_DEFAULT == ON);
  #endif

  if (MFLIP_AUTOMATIC_MEMORY != ON || !transform.meridianFlips) settings.meridianFlipAuto = (MFLIP_AUTOMATIC_DEFAULT == ON);

  #if PIER_SIDE_PREFERRED_MEMORY != ON
    settings.preferredPierSide = (PierSideSelect)PIER_SIDE_PREFERRED_DEFAULT;
  #endif

  // calculate base and current maximum step rates
  #if GOTO_FEATURE == ON
    usPerStepBase = 1000000.0F/((axis1.getStepsPerMeasure()/RAD_DEG_RATIO)*SLEW_RATE_BASE_DESIRED);
    #if SLEW_RATE_MEMORY != ON
      settings.usPerStepCurrent = usPerStepBase;
    #endif
  #else
    usPerStepBase = 1000000.0F/((axis1.getStepsPerMeasure()/RAD_DEG_RATIO)*1.0F);
    settings.usPerStepCurrent = usPerStepBase;
  #endif
  if (usPerStepBase < usPerStepLowerLimit()) usPerStepBase = usPerStepLowerLimit()*2.0F;
  if (settings.usPerStepCurrent > 1000000.0F) settings.usPerStepCurrent = usPerStepBase;
  if (settings.usPerStepCurrent < usPerStepBase/2.0F) settings.usPerStepCurrent = usPerStepBase/2.0F;
  if (settings.usPerStepCurrent > usPerStepBase*2.0F) settings.usPerStepCurrent = usPerStepBase*2.0F;

  updateAccelerationRates();
}

// goto to equatorial target position (Native coordinate system) using the defaut preferredPierSide
CommandError Goto::request() {
  return request(target, settings.preferredPierSide);
}

#if GOTO_FEATURE == ON

// goto equatorial position (Native or Mount coordinate system)
CommandError Goto::request(Coordinate coords, PierSideSelect pierSideSelect, bool native) {

  if (native) {
    coords.pierSide = PIER_SIDE_NONE;
    transform.nativeToMount(&coords);
  }

  CommandError e = setTarget(&coords, pierSideSelect);
  if (e == CE_SLEW_IN_SLEW) { stop(); return e; }
  if (e != CE_NONE) return e;

  lastAlignTarget = target;

  // handle special case of a tangent arm mount
  #if AXIS2_TANGENT_ARM == ON
    double a1, a2;
    transform.mountToInstrument(&target, &a1, &a2);
    a2 = a2 - axis2.getIndexPosition();
    if (a2 < axis2.settings.limits.min) return CE_SLEW_ERR_OUTSIDE_LIMITS;
    if (a2 > axis2.settings.limits.max) return CE_SLEW_ERR_OUTSIDE_LIMITS;
  #endif

  limits.enabled(true);
  mount.syncToEncoders(false);
  if (firstGoto) {
    mount.tracking(true);
    firstGoto = false;
  }
  guide.backlashEnableControl(true);

  // allow slewing near target for Eq modes if not too close to the poles
  slewDestinationDistHA = 0.0;
  slewDestinationDistDec = 0.0;
  if (transform.mountType != ALTAZM && 
      park.state != PS_PARKING &&
      home.state != HS_HOMING &&
      fabs(target.d) < Deg90 - degToRad(GOTO_OFFSET)) {
    slewDestinationDistHA = degToRad(GOTO_OFFSET);
    slewDestinationDistDec = degToRad(GOTO_OFFSET);
    if (target.pierSide == PIER_SIDE_WEST) slewDestinationDistDec = -slewDestinationDistDec;
  }

  // prepare for goto
  Coordinate current = mount.getMountPosition(CR_MOUNT_HOR);
  state = GS_GOTO;
  stage = GG_NEAR_DESTINATION;
  start = current;
  destination = target;
  nearDestinationRefineStages = 1;

  // add waypoint if needed
  if (transform.mountType != ALTAZM && MFLIP_SKIP_HOME == OFF && start.pierSide != destination.pierSide) {
    VLF("MSG: Mount, goto changes pier side, setting waypoint at home");
    waypoint(&current);
  }

  // start the goto monitor
  if (taskHandle != 0) tasks.remove(taskHandle);
  taskHandle = tasks.add(0, 0, true, 3, gotoWrapper, "MntGoto");
  if (taskHandle) {
    VLF("MSG: Mount, create goto monitor task (idle, priority 3)... success");
    VLF("MSG: Mount, starting goto");

    e = startAutoSlew();
    if (e != CE_NONE) return e;

    tasks.setPeriodMicros(taskHandle, FRACTIONAL_SEC_US);
    VF("MSG: Mount, goto monitor task set rate "); V(FRACTIONAL_SEC_US); VL("us");

    mountStatus.sound.alert();

  } else { DLF("WRN: Mount, start goto monitor task... FAILED!"); }

  return CE_NONE;
}
#else

// sync replaces goto to equatorial position (Native or Mount coordinate system) when GOTO_FEATURE is OFF
CommandError Goto::request(Coordinate coords, PierSideSelect pierSideSelect, bool native) {
  mountStatus.sound.alert();

  CommandError result = requestSync(coords, pierSideSelect, native);

  // check if parking and mark as finished or unparked as needed
  if (park.state == PS_PARKING) park.requestDone();

  // check if homing
  if (home.state == HS_HOMING) home.requestDone();

  return result;
}
#endif

// sync to equatorial target position (Native coordinate system) using the default preferredPierSide
CommandError Goto::requestSync() {
  return requestSync(target, settings.preferredPierSide);
}

// sync to equatorial position (Native or Mount coordinate system)
CommandError Goto::requestSync(Coordinate coords, PierSideSelect pierSideSelect, bool native) {
  
  if (native) {
    coords.pierSide = PIER_SIDE_NONE;
    transform.nativeToMount(&coords);
  }

  CommandError e = setTarget(&coords, pierSideSelect, false);
  if (e != CE_NONE) return e;
  
  if (mount.isHome()) mount.tracking(true);

  double a1, a2;
  transform.mountToInstrument(&target, &a1, &a2);
  axis1.setInstrumentCoordinate(a1);
  axis2.setInstrumentCoordinate(a2);

  limits.enabled(true);
  mount.syncToEncoders(true);

  VLF("MSG: Mount, sync instrument coordinates updated");

  return CE_NONE;
}

// checks for valid target and determines pier side (Mount coordinate system)
CommandError Goto::setTarget(Coordinate *coords, PierSideSelect pierSideSelect, bool isGoto) {

  CommandError e = validate();
  if (e == CE_SLEW_ERR_IN_STANDBY && (mount.isHome() || AXIS1_SYNC_THRESHOLD != OFF || AXIS1_TARGET_TOLERANCE != OFF)) {
    mount.enable(true);
    e = validate();
  }
  if (e == CE_NONE && isGoto && limits.isAboveOverhead()) e = CE_SLEW_ERR_OUTSIDE_LIMITS;
  if (e != CE_NONE) return e;

  target = *coords;

  if (transform.mountType == ALTAZM) transform.horToEqu(&target); else transform.equToHor(&target);

  // east side of pier is always the default polar-home position
  // east side of pier - we're in the western sky and the HA's are positive
  // west side of pier - we're in the eastern sky and the HA's are negative

  Coordinate current = mount.getMountPosition(CR_MOUNT);

  target.pierSide = current.pierSide;
  e = limits.validateTarget(&target);
  if (e != CE_NONE) return e;

  if (transform.meridianFlips) {
    double a1;
    if (transform.mountType == ALTAZM) a1 = target.z; else a1 = target.h;

    double a1e = a1, a1w = a1;

    bool pastMeridianE = a1 < -limits.settings.pastMeridianE;
    bool pastMeridianW = a1 > limits.settings.pastMeridianW;
    if (pastMeridianE) a1e += Deg360;                          // range 0 to 360 degrees, east of pier
    if (pastMeridianW) a1w -= Deg360;                          // range 0 to -360 degrees, west of pier
    bool pastMaxE = a1e > axis1.settings.limits.max;
    bool pastMinW = a1w < axis1.settings.limits.min;

    if (mount.isHome()) {
      VL("MSG: Mount, set target from home");
      if (transform.mountType == FORK) {
        if (settings.preferredPierSide == PSS_WEST) target.pierSide = PIER_SIDE_WEST; else target.pierSide = PIER_SIDE_EAST;
      } else {
        if (a1 < 0) target.pierSide = PIER_SIDE_WEST; else target.pierSide = PIER_SIDE_EAST;
      }
    } else
    if (pierSideSelect == PSS_EAST || pierSideSelect == PSS_EAST_ONLY) {
      target.pierSide = PIER_SIDE_EAST;
      if (pastMeridianE && pastMaxE) {
        VF("MSG: Mount, set target EAST TO WEST: ");
        target.pierSide = PIER_SIDE_WEST;
      } else { VF("MSG: Mount, set target EAST stays EAST: !("); }
      V(radToDeg(a1)); V(" < "); V(-radToDeg(limits.settings.pastMeridianE)); V(" && "); V(radToDeg(a1e)); V(" > "); V(radToDeg(axis1.settings.limits.max)); VL(")");
      if (pastMeridianE && !pastMaxE) a1 = a1e;
    } else
    if (pierSideSelect == PSS_WEST || pierSideSelect == PSS_WEST_ONLY) {
      target.pierSide = PIER_SIDE_WEST;
      VLF("MSG: Mount, set target ");
      if (pastMeridianW && pastMinW) {
        VF("MSG: Mount, set target WEST TO EAST: (");
        target.pierSide = PIER_SIDE_EAST;
      } else { VF("MSG: Mount, set target WEST stays WEST: !(");  }
      V(radToDeg(a1)); V(" > "); V(radToDeg(limits.settings.pastMeridianW)); V(" && "); V(radToDeg(a1w)); V(" < "); V(radToDeg(axis1.settings.limits.min)); VL(")");
      if (pastMeridianW && !pastMinW) a1 = a1w;
    } else
    if (pierSideSelect == PSS_BEST || pierSideSelect == PSS_SAME_ONLY) {
      if (current.pierSide == PIER_SIDE_EAST) { 
        if (pastMeridianE && pastMaxE) {
          VF("MSG: Mount, set target BEST EAST TO WEST: (");
          target.pierSide = PIER_SIDE_WEST;
        } else { VF("MSG: Mount, set target BEST stays EAST: !("); }
        V(radToDeg(a1)); V(" < "); V(-radToDeg(limits.settings.pastMeridianE)); V(" && "); V(radToDeg(a1e)); V(" > "); V(radToDeg(axis1.settings.limits.max)); VL(")");
        if (pastMeridianE && !pastMaxE) a1 = a1e;
      }
      if (current.pierSide == PIER_SIDE_WEST) {
          if (pastMeridianW && pastMinW) {
          VF("MSG: Mount, set target BEST WEST TO EAST: (");
          target.pierSide = PIER_SIDE_EAST;
        } else { VF("MSG: Mount, set target BEST stays WEST: !("); }
        V(radToDeg(a1)); V(" > "); V(radToDeg(limits.settings.pastMeridianW)); V(" && "); V(radToDeg(a1w)); V(" < "); V(radToDeg(axis1.settings.limits.min)); VL(")");
        if (pastMeridianW && !pastMinW) a1 = a1w;
      }
    }

    if (target.pierSide == PIER_SIDE_EAST) {
      VF("MSG: Mount, set target final EAST (a1="); V(radToDeg(a1)); VL(")");
    } else
    if (target.pierSide == PIER_SIDE_WEST) {
      VF("MSG: Mount, set target final WEST (a1="); V(radToDeg(a1)); VL(")");
    }

    if (transform.mountType == ALTAZM) target.z = a1; else target.h = a1;

    if (pierSideSelect == PSS_EAST_ONLY && target.pierSide != PIER_SIDE_EAST) return CE_SLEW_ERR_OUTSIDE_LIMITS; else
    if (pierSideSelect == PSS_WEST_ONLY && target.pierSide != PIER_SIDE_WEST) return CE_SLEW_ERR_OUTSIDE_LIMITS; else
    if (pierSideSelect == PSS_SAME_ONLY && target.pierSide != current.pierSide) return CE_SLEW_ERR_OUTSIDE_LIMITS;
  } else {
    target.pierSide = PIER_SIDE_EAST;
  }

  if (target.pierSide != PIER_SIDE_WEST) target.pierSide = PIER_SIDE_EAST;

  if (transform.mountType == ALTAZM) transform.horToEqu(&target); else transform.equToHor(&target);
  transform.observedPlaceToMount(&target);
  transform.hourAngleToRightAscension(&target, false);

  return CE_NONE;
}

// stop any presently active goto
void Goto::stop() {
  if (state == GS_GOTO && stage > GG_READY_ABORT) stage = GG_READY_ABORT;
}

// general status checks ahead of sync or goto
CommandError Goto::validate() {
  if (axis1.fault())           return CE_SLEW_ERR_HARDWARE_FAULT;
  if (axis2.fault())           return CE_SLEW_ERR_HARDWARE_FAULT;
  if (!axis1.isEnabled())      return CE_SLEW_ERR_IN_STANDBY;
  if (!axis2.isEnabled())      return CE_SLEW_ERR_IN_STANDBY;
  if (park.state == PS_PARKED) return CE_SLEW_ERR_IN_PARK;
  if (state != GS_NONE)        return CE_SLEW_IN_SLEW;
  if (guide.state != GU_NONE)  return CE_SLEW_IN_MOTION;
  if (mount.isSlewing())       return CE_SLEW_IN_MOTION;
  if (limits.isGotoError())    return CE_SLEW_ERR_OUTSIDE_LIMITS;
  return CE_NONE;
}

// add an align star (at the current position relative to target)
CommandError Goto::alignAddStar() {
  if (alignState.currentStar > alignState.lastStar) return CE_PARAM_RANGE;

  CommandError e = CE_NONE;

  // first star, get ready for a new pointing model, init/sync then call gta.addStar 
  if (alignState.currentStar == 1) {
    #if ALIGN_MAX_NUM_STARS > 1  
      transform.align.init(transform.mountType, site.location.latitude);
    #endif
    e = requestSync(gotoTarget, settings.preferredPierSide);
  }

  // add an align star
  if (e == CE_NONE) {
    Coordinate mountPosition = mount.getMountPosition(CR_MOUNT_ALL);

    // update the targets HA and Horizon coords as necessary
    transform.rightAscensionToHourAngle(&lastAlignTarget, true);
    if (transform.mountType == ALTAZM) transform.equToHor(&lastAlignTarget);

    #if ALIGN_MAX_NUM_STARS > 1
      e = transform.align.addStar(alignState.currentStar, alignState.lastStar, &lastAlignTarget, &mountPosition);
    #endif
    if (e == CE_NONE) alignState.currentStar++;
  }

  return e;
}

// reset the alignment model
void Goto::alignReset() {
  alignState.currentStar = 0;
  alignState.lastStar = 0;
  #if ALIGN_MAX_NUM_STARS > 1
    transform.align.modelClear();
  #endif
}

#if GOTO_FEATURE == ON
// set any additional destinations required for a goto
void Goto::waypoint(Coordinate *current) {
  // HA goes from +90...0..-90
  //                W   .   E
  // meridian flip, only happens for equatorial mounts

  stage = GG_WAYPOINT_HOME;

  // default goes straight to the home position
  destination = home.position;

  // if the home position is at 0 hours, we're done
  if (home.position.h == 0.0) return;

  double d60 = degToRad(120);
  double d45 = degToRad(135);
  if (current->pierSide == PIER_SIDE_EAST) { d60 = Deg180 - d60; d45 = Deg180 - d45; }

  // decide if we should first move to 60 deg. HA (4 hours) to get away from the horizon limits
  if (current->a < Deg10 && fabs(start.h) > Deg90) { destination.h = d60; stage = GG_WAYPOINT_AVOID; return; }

  // decide if we should first move to 45 deg. HA (3 hours) to get away from the horizon limits
  // if at a low latitude and in the opposite sky, |HA| = 6 is very low on the horizon and we need
  // to delay arriving there during a meridian flip.  In the extreme case, where the user is very
  // near the Earths equator an Horizon limit of -10 or -15 may be necessary for proper operation
  if (current->a < Deg20 && site.locationEx.latitude.absval < Deg45) {
    if (site.location.latitude >= 0) {
      if (current->d <= Deg90 - site.location.latitude) { destination.h = d45; stage = GG_WAYPOINT_AVOID; }
    } else {
      if (current->d >= -Deg90 - site.location.latitude) { destination.h = d45; stage = GG_WAYPOINT_AVOID; }
    }
  }
}

// monitor goto
void Goto::poll() {
  if (stage == GG_READY_ABORT) {
    VLF("MSG: Mount, goto abort requested");
    stage = GG_ABORT;
    meridianFlipHome.paused = false;
    meridianFlipHome.resume = false;
    axis1.autoSlewAbort();
    axis2.autoSlewAbort();
  }

  if (!mount.isSlewing()) {
    if (stage == GG_WAYPOINT_AVOID) {
      VLF("MSG: Mount, goto waypoint reached");
      stage = GG_WAYPOINT_HOME;
      destination = home.position;
      startAutoSlew();
    } else

    if (stage == GG_WAYPOINT_HOME) {
      if (settings.meridianFlipPause && !meridianFlipHome.resume) { meridianFlipHome.paused = true; goto skip; }
      meridianFlipHome.paused = false;
      meridianFlipHome.resume = false;

      VLF("MSG: Mount, goto home reached");
      stage = GG_NEAR_DESTINATION;
      destination = target;
      startAutoSlew();
    } else

    if (stage == GG_NEAR_DESTINATION) {
      if (slewDestinationDistHA != 0.0 || transform.mountType == ALTAZM) {

        if (transform.mountType != ALTAZM || !nearDestinationRefineStages) stage = GG_DESTINATION;
        nearDestinationRefineStages--;

        VLF("MSG: Mount, goto near destination reached");
        destination = target;
        if (!alignActive() || GOTO_OFFSET_ALIGN == OFF) {
          slewDestinationDistHA = 0.0;
          slewDestinationDistDec = 0.0;
        }
        startAutoSlew();
      } else {
        stage = GG_DESTINATION;
        VLF("MSG: Mount, goto near destination skipped");
      }
    } else

    if (stage == GG_DESTINATION || stage == GG_ABORT) {
      VLF("MSG: Mount, goto destination reached");
      state = GS_NONE;
      mount.update();

      // kill this monitor
      tasks.setDurationComplete(taskHandle);
      taskHandle = 0;
      VLF("MSG: Mount, goto monitor task terminated");

      // check if parking and mark as finished or unparked as needed
      if (park.state == PS_PARKING) {
        if (stage == GG_ABORT) park.requestAborted(); else park.requestDone();
      }

      // check if homing
      if (home.state == HS_HOMING) {
        if (stage == GG_ABORT) home.requestAborted(); else home.requestDone();
      }

      // reset goto stage
      stage = GG_NONE;

      mountStatus.sound.alert();

      return;
    }
  }

  skip:

  // keep updating the axis targets to match the mount target
  // but allow timeout to stop tracking to guarantee synchronization
  if (!axis1.nearTarget() || !axis2.nearTarget()) nearTargetTimeout = millis();

  if (mount.isTracking()) {
    target.r += siderealToRad(mount.trackingRateOffsetRA)/FRACTIONAL_SEC;
    target.d += siderealToRad(mount.trackingRateOffsetDec)/FRACTIONAL_SEC;
    transform.rightAscensionToHourAngle(&target, false);
    if (stage == GG_NEAR_DESTINATION || stage == GG_DESTINATION) {
      if (millis() - nearTargetTimeout < 4000) {
        Coordinate nearTarget = target;
        nearTarget.h -= slewDestinationDistHA;
        nearTarget.d -= slewDestinationDistDec;

        if (transform.mountType == ALTAZM) transform.equToHor(&nearTarget);
        double a1, a2;
        transform.mountToInstrument(&nearTarget, &a1, &a2);

        axis1.setTargetCoordinate(a1);
        axis2.setTargetCoordinate(a2);
      }
    }
  }
}

// start slews with approach correction and parking support
CommandError Goto::startAutoSlew() {
  CommandError e;

  if (stage == GG_NEAR_DESTINATION || stage == GG_DESTINATION) {
    destination.h -= slewDestinationDistHA;
    destination.d -= slewDestinationDistDec;
  }

  double a1, a2;
  transform.mountToInstrument(&destination, &a1, &a2);

  if (stage == GG_DESTINATION && park.state == PS_PARKING) {
    axis1.setTargetCoordinatePark(a1);
    axis2.setTargetCoordinatePark(a2);
  } else {
    axis1.setTargetCoordinate(a1);
    axis2.setTargetCoordinate(a2);
  }

  VF("MSG: Mount, goto target coordinates set (a1="); V(radToDeg(a1)); VF("deg, a2="); V(radToDeg(a2)); VLF(" deg)");

  e = axis1.autoGoto(radsPerSecondCurrent);
  if (e == CE_NONE) e = axis2.autoGoto(radsPerSecondCurrent*((float)(AXIS2_SLEW_RATE_PERCENT)/100.0F));

  nearTargetTimeout = millis();

  return e;
}
#endif

// update acceleration rates for goto and guiding
void Goto::updateAccelerationRates() {
  radsPerSecondCurrent = (1000000.0F/settings.usPerStepCurrent)/(float)axis1.getStepsPerMeasure();
  rate = radsPerSecondCurrent;
  #if GOTO_FEATURE == ON
    float secondsToAccelerate = (degToRadF((float)(SLEW_ACCELERATION_DIST))/radsPerSecondCurrent)*2.0F;
    float secondsToAccelerateAbort = (degToRadF((float)(SLEW_RAPID_STOP_DIST))/radsPerSecondCurrent)*2.0F;
  #else
    float secondsToAccelerate = (degToRadF((float)(5.0F))/radsPerSecondCurrent)*2.0F;
    float secondsToAccelerateAbort = (degToRadF((float)(2.0F))/radsPerSecondCurrent)*2.0F;
  #endif
  axis1.setSlewAccelerationRate(radsPerSecondCurrent/secondsToAccelerate);
  axis1.setSlewAccelerationRateAbort(radsPerSecondCurrent/secondsToAccelerateAbort);
  axis2.setSlewAccelerationRate(radsPerSecondCurrent/secondsToAccelerate);
  axis2.setSlewAccelerationRateAbort(radsPerSecondCurrent/secondsToAccelerateAbort);
}

// estimate average microseconds per step lower limit
float Goto::usPerStepLowerLimit() {
  // basis is platform/clock-rate specific (for square wave)
  float r_us = HAL_MAXRATE_LOWER_LIMIT;
  
  // higher speed ISR code path?
  #if STEP_WAVE_FORM == PULSE || STEP_WAVE_FORM == DEDGE
    r_us /= 1.6F;
  #endif

  // average required goto us rates for each axis with any micro-step mode switching applied
  float r_us_axis1 = r_us/axis1.getStepsPerStepSlewing();
  float r_us_axis2 = r_us/axis2.getStepsPerStepSlewing();

  // average in axis2 step rate scaling for drives where the reduction ratio isn't equal
  r_us = (1.0F/(1.0F/r_us_axis1 + 1.0F/r_us_axis2))*2.0F;

  // return rate in us units
  return r_us;
}

Goto goTo;

#endif
