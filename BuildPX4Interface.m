%*******************************************************************************
% File:
% PX4Main.c
%
% Description:
% Builds a Simulink S-Function named PX4 by compiling the PX4Main.c.
% When changes are made to PX4Main.c or PX4Interface.h this script must be run 
% to recompile the changes before they can be used by the Simulink S-function.
%
% Author:
% Michael Skadan
%
%*******************************************************************************
% Notices:
% Copyright 2018, United States Government as represented by the Administrator
% of the National Aeronautics and Space Administration. All Rights Reserved.
%
% MAVLINK__________________________________________
% Portions of this software were generated using Mavlink
% (https://github.com/mavlink) and are subject to the following MIT License:
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of the generated software (the "Generated Software"), to deal in the Generated
% Software without restriction, including without limitation the rights to use,
% copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
% the Generated Software, and to permit persons to whom the Generated Software
% is furnished to do so, subject to the following conditions:
% The above copyright notice and this permission notice shall be included in all
% copies or substantial portions of the Generated Software.
%
% THE GENERATED SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
% EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
% MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO
% EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES
% OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
% ARISING FROM, OUT OF OR IN CONNECTION WITH THE GENERATED SOFTWARE OR THE USE
% OR OTHER DEALINGS IN THE GENERATED SOFTWARE.
%
% NASA Disclaimers
% No Warranty:
% THE SUBJECT SOFTWARE IS PROVIDED "AS IS" WITHOUT ANY WARRANTY OF ANY KIND,
% EITHER EXPRESSED, IMPLIED, OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, ANY
% WARRANTY THAT THE SUBJECT SOFTWARE WILL CONFORM TO SPECIFICATIONS, ANY IMPLIED
% WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, OR FREEDOM
% FROM INFRINGEMENT, ANY WARRANTY THAT THE SUBJECT SOFTWARE WILL BE ERROR FREE,
% OR ANY WARRANTY THAT DOCUMENTATION, IF PROVIDED, WILL CONFORM TO THE SUBJECT
% SOFTWARE. THIS AGREEMENT DOES NOT, IN ANY MANNER, CONSTITUTE AN ENDORSEMENT BY
% GOVERNMENT AGENCY OR ANY PRIOR RECIPIENT OF ANY RESULTS, RESULTING DESIGNS,
% HARDWARE, SOFTWARE PRODUCTS OR ANY OTHER APPLICATIONS RESULTING FROM USE OF
% THE SUBJECT SOFTWARE.  FURTHER, GOVERNMENT AGENCY DISCLAIMS ALL WARRANTIES AND
% LIABILITIES REGARDING THIRD-PARTY SOFTWARE, IF PRESENT IN THE ORIGINAL
% SOFTWARE, AND DISTRIBUTES IT "AS IS."
%
% Waiver and Indemnity:
% RECIPIENT AGREES TO WAIVE ANY AND ALL CLAIMS AGAINST THE UNITED STATES
% GOVERNMENT, ITS CONTRACTORS AND SUBCONTRACTORS, AS WELL AS ANY PRIOR
% RECIPIENT.  IF RECIPIENT'S USE OF THE SUBJECT SOFTWARE RESULTS IN ANY
% LIABILITIES, DEMANDS, DAMAGES, EXPENSES OR LOSSES ARISING FROM SUCH USE,
% INCLUDING ANY DAMAGES FROM PRODUCTS BASED ON, OR RESULTING FROM, RECIPIENT'S
% USE OF THE SUBJECT SOFTWARE, RECIPIENT SHALL INDEMNIFY AND HOLD HARMLESS THE
% UNITED STATES GOVERNMENT, ITS CONTRACTORS AND SUBCONTRACTORS, AS WELL AS ANY
% PRIOR RECIPIENT, TO THE EXTENT PERMITTED BY LAW.  RECIPIENT'S SOLE REMEDY FOR
% ANY SUCH MATTER SHALL BE THE IMMEDIATE, UNILATERAL TERMINATION OF THIS
% AGREEMENT.
%
%*******************************************************************************

def = legacy_code('initialize')
def.SFunctionName = 'PX4'
def.HeaderFiles = {'PX4Interface.h'}
def.SourceFiles = {'PX4Main.c'}
def.StartFcnSpec = 'start()'
def.OutputFcnSpec = 'step(uint32 u1[1], double u2[13], double u3[12], uint16 u4[20], double y1[16])'
legacy_code('sfcn_cmex_generate', def)
legacy_code('compile', def)
