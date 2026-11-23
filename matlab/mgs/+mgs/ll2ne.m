function [n, e] = ll2ne(lat, lon, tlat, tlon)
%MGS.LL2NE  lat/lon -> the target-tangent plane tot_node actually uses.
%
%   [n, e] = mgs.ll2ne(lat, lon, tlat, tlon)
%
%   Returns the position of (lat,lon) relative to the target (tlat,tlon), in
%   metres, north and east positive.
%
% ---------------------------------------------------------------------------
%   THIS IS DELIBERATELY NOT WGS84.
%
%   tot_node.py:2713-2714 is a flat-earth approximation with a single constant:
%
%       dn = (T_lat - p_lat) * 111320.0
%       de = (T_lon - p_lon) * 111320.0 * cos(radians(p_lat))
%
%   and geodesy.py:4 declares EARTH_M_PER_DEG = 111320.0 for the same purpose.
%   A proper meridian/prime-vertical conversion here would be MORE accurate and
%   WRONG: the model would then be working in a slightly different frame from
%   the vehicle, and mgs.shadowDiff would report a steady offset that is a
%   coordinate artefact rather than a divergence.  Matching the flying code
%   beats being right about the geoid.
%
%   Note the cos() takes the AIRCRAFT's latitude, not the target's -- that is
%   what tot_node.py:2714 does (`p[0]`, the aircraft), and at our ranges the
%   difference is real.
%
%   Sign: tot_node computes target-minus-aircraft (a vector pointing AT the
%   target).  This function returns aircraft-minus-target, because that is the
%   convention of the model state (target at the origin).  stot_guidance then
%   negates it back on line 2475-2476.  The two conventions are one sign apart
%   and mixing them silently mirrors the whole engagement, so it is stated here
%   rather than left to be rediscovered.
%
%   See also MGS.SHADOWDIFF, MGS.PARSESTREAM.

EARTH_M_PER_DEG = 111320.0;                 % geodesy.py:4, tot_node.py:51

n = (lat - tlat) * EARTH_M_PER_DEG;
e = (lon - tlon) * EARTH_M_PER_DEG .* cos(deg2rad(lat));
end
