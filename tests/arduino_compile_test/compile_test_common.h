// Copyright 2026 mjbots Robotic Systems, LLC.  info@mjbots.com
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// @file
///
/// Exercises the complete user-facing API of MoteusController using
/// only the public spellings a sketch would use
/// (e.g. Moteus::OutputExact::Command).  This function is explicitly
/// instantiated by the compile tests but never executed — its only
/// purpose is to fail the build if any part of the API stops
/// compiling.

#pragma once

template <typename Moteus>
void ExerciseMoteusApi(Moteus& moteus) {
  typename Moteus::Query::Format query_format;
  query_format.trajectory_complete = Moteus::kInt8;

  typename Moteus::CanFdFrame frame;

  // Query
  frame = moteus.MakeQuery();
  frame = moteus.MakeQuery(&query_format);
  (void)moteus.SetQuery();
  (void)moteus.SetQuery(&query_format);

  // StopMode
  frame = moteus.MakeStop();
  (void)moteus.SetStop();
  moteus.BeginStop();

  // BrakeMode
  frame = moteus.MakeBrake();
  (void)moteus.SetBrake();
  moteus.BeginBrake();

  // PositionMode
  typename Moteus::PositionMode::Command position_cmd;
  position_cmd.position = 0.0;
  typename Moteus::PositionMode::Format position_fmt;
  frame = moteus.MakePosition(position_cmd);
  frame = moteus.MakePosition(position_cmd, &position_fmt, &query_format);
  (void)moteus.SetPosition(position_cmd);
  moteus.BeginPosition(position_cmd);
  (void)moteus.SetPositionWaitComplete(position_cmd, 0.05);

  // VFOCMode
  typename Moteus::VFOCMode::Command vfoc_cmd;
  typename Moteus::VFOCMode::Format vfoc_fmt;
  frame = moteus.MakeVFOC(vfoc_cmd);
  frame = moteus.MakeVFOC(vfoc_cmd, &vfoc_fmt, &query_format);
  (void)moteus.SetVFOC(vfoc_cmd);
  moteus.BeginVFOC(vfoc_cmd);

  // CurrentMode
  typename Moteus::CurrentMode::Command current_cmd;
  typename Moteus::CurrentMode::Format current_fmt;
  frame = moteus.MakeCurrent(current_cmd);
  frame = moteus.MakeCurrent(current_cmd, &current_fmt, &query_format);
  (void)moteus.SetCurrent(current_cmd);
  moteus.BeginCurrent(current_cmd);

  // StayWithinMode
  typename Moteus::StayWithinMode::Command stay_within_cmd;
  typename Moteus::StayWithinMode::Format stay_within_fmt;
  frame = moteus.MakeStayWithin(stay_within_cmd);
  frame = moteus.MakeStayWithin(stay_within_cmd, &stay_within_fmt,
                                &query_format);
  (void)moteus.SetStayWithin(stay_within_cmd);
  moteus.BeginStayWithin(stay_within_cmd);

  // ZeroVelocityMode
  typename Moteus::ZeroVelocityMode::Command zero_velocity_cmd;
  frame = moteus.MakeZeroVelocity();
  frame = moteus.MakeZeroVelocity(zero_velocity_cmd);
  (void)moteus.SetZeroVelocity();
  moteus.BeginZeroVelocity();

  // GpioRead
  typename Moteus::GpioRead::Command gpio_cmd;
  frame = moteus.MakeGpioRead();
  frame = moteus.MakeGpioRead(gpio_cmd);
  (void)moteus.SetGpioRead();
  moteus.BeginGpioRead();

  // AuxPwmWrite
  typename Moteus::AuxPwmWrite::Command aux_pwm_cmd;
  frame = moteus.MakeAuxPwmWrite(aux_pwm_cmd);
  (void)moteus.SetAuxPwmWrite(aux_pwm_cmd);
  moteus.BeginAuxPwmWrite(aux_pwm_cmd);

  // OutputNearest
  typename Moteus::OutputNearest::Command output_nearest_cmd;
  output_nearest_cmd.position = 0.0;
  frame = moteus.MakeOutputNearest(output_nearest_cmd);
  (void)moteus.SetOutputNearest(output_nearest_cmd);
  moteus.BeginOutputNearest(output_nearest_cmd);

  // OutputExact
  typename Moteus::OutputExact::Command output_exact_cmd;
  output_exact_cmd.position = 0.0;
  frame = moteus.MakeOutputExact(output_exact_cmd);
  (void)moteus.SetOutputExact(output_exact_cmd);
  moteus.BeginOutputExact(output_exact_cmd);

  // RequireReindex
  typename Moteus::RequireReindex::Command require_reindex_cmd;
  frame = moteus.MakeRequireReindex(require_reindex_cmd);
  (void)moteus.SetRequireReindex(require_reindex_cmd);
  moteus.BeginRequireReindex(require_reindex_cmd);

  // RecapturePositionVelocity
  typename Moteus::RecapturePositionVelocity::Command recapture_cmd;
  frame = moteus.MakeRecapturePositionVelocity(recapture_cmd);
  (void)moteus.SetRecapturePositionVelocity(recapture_cmd);
  moteus.BeginRecapturePositionVelocity(recapture_cmd);

  // Non-command methods.
  (void)moteus.Poll();
  (void)moteus.BeginSingleCommand(frame);
  (void)moteus.ExecuteSingleCommand(frame);

  const typename Moteus::Result& result = moteus.last_result();
  (void)result.timestamp;
  (void)result.frame;
  (void)result.values.position;

#ifdef ARDUINO
  // Diagnostic channel operations (Arduino String based).
  (void)moteus.DiagnosticCommand(String("d stop"));
  (void)moteus.DiagnosticCommand(String("conf get id.id"),
                                 Moteus::kExpectSingleLine);
  (void)moteus.SetDiagnosticRead();
  (void)moteus.SetDiagnosticRead(1);
  moteus.SetDiagnosticFlush();
  moteus.SetDiagnosticFlush(1);
#endif
}
