// This code is a direct adaptation of the original code written in C99
// https://github.com/yinqiwen/geohash-int

/*
 *Copyright (c) 2013-2014, yinqiwen <yinqiwen@gmail.com>
 *All rights reserved.
 *
 *Redistribution and use in source and binary forms, with or without
 *modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of Redis nor the names of its contributors may be used
 *    to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 *THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 *BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 *THE POSSIBILITY OF SUCH DAMAGE.
 */

import 'dart:math' as Math;
import 'dart:typed_data';
import 'package:quiver/collection.dart';

enum GeoDirection {
    GEOHASH_NORTH,
    GEOHASH_EAST,
    GEOHASH_WEST,
    GEOHASH_SOUTH,
    GEOHASH_SOUTH_WEST,
    GEOHASH_SOUTH_EAST,
    GEOHASH_NORTH_WEST,
    GEOHASH_NORTH_EAST
}

enum GeoProjection {
    WGS84,
    Mercator,
}

enum GeoCoordinate {
    Latitude,
    Longitude,
}

const double _EPSILON = 0.000001;

class GeoHashBits {
    int bits;
    int step;
    GeoHashBits(this.bits, this.step);
    GeoHashBits clone() => GeoHashBits(this.bits, this.step);
    @override
    operator==(other) =>
        identical(this, other) ||
        other is GeoHashBits
        && bits == other.bits
        && step == other.step;
    @override
    String toString() => "GeoHash($bits,$step)";
}

class GeoHashRange {
    double max;
    double min;
    static const MERCATOR_MAX = 20037726.37;
    static const WGS84_LAT_MAX = 85.05113;
    static const WGS84_LAT_MIN = -85.05113;
    static const WGS84_LON_MAX = 180.0;
    static const WGS84_LON_MIN = -180.0;

    GeoHashRange(this.min, this.max);
    GeoHashRange clone() => GeoHashRange(this.min, this.max);

    bool within(double value, {bool inclusive: false}) =>
        inclusive? value >= min && value <= max :
                   value > min && value < max;

    //ignore: missing_return
    factory GeoHashRange.getStandardRange(GeoProjection proj, GeoCoordinate coord) {
        switch(proj) {
            case GeoProjection.WGS84:
                switch(coord) {
                    case GeoCoordinate.Latitude:
                        return GeoHashRange(WGS84_LAT_MIN,WGS84_LAT_MAX);
                    case GeoCoordinate.Longitude:
                        return GeoHashRange(WGS84_LON_MIN,WGS84_LON_MAX);
                }
                break;
            case GeoProjection.Mercator:
                return GeoHashRange(-MERCATOR_MAX,MERCATOR_MAX);
        }
    }
    @override
    operator==(other) =>
        identical(this, other) ||
        other is GeoHashRange
        && (max - other.max).abs() < _EPSILON
        && (min - other.min).abs() < _EPSILON;
    @override
    String toString() => "GeoHashRange($min,$max)";
}

class GeoHashArea {
    GeoHashBits hash;
    GeoHashRange latitude;
    GeoHashRange longitude;
    GeoHashArea(this.hash, this.latitude, this.longitude);
    GeoHashArea clone() => GeoHashArea(this.hash.clone(), this.latitude.clone(),
                                       this.longitude.clone());

    bool within(double lat, double lon, {bool inclusive: false}) =>
        latitude.within(lat) && longitude.within(lon);

    @override
    operator==(other) =>
        identical(this, other) ||
        other is GeoHashArea
        && hash == other.hash
        && latitude == other.latitude
        && longitude == other.longitude;
    @override
    String toString() => "GeoHashArea($hash, $latitude, $longitude)";
}

class GeoHashNeighbors {
    GeoHashBits north;
    GeoHashBits east;
    GeoHashBits west;
    GeoHashBits south;
    GeoHashBits north_east;
    GeoHashBits south_east;
    GeoHashBits north_west;
    GeoHashBits south_west;
    GeoHashNeighbors({
        this.north, this.east, this.west, this.south,
        this.north_east, this.south_east, this.north_west, this.south_west
    });
}


// Return null on failure
GeoHashBits geohash_encode(GeoHashRange lat_range, GeoHashRange lon_range,
        double latitude, double longitude, int step) {

    if(step > 32 || step <= 0)
        return null;

    if(latitude < lat_range.min || latitude > lat_range.max ||
       longitude < lon_range.min || longitude > lon_range.max)
       return null;

    var hash = GeoHashBits(0, step);

    var lat_r = lat_range.clone();
    var lon_r = lon_range.clone();

    for(int i = 0; i < step; i++) {
        int lat_bit, lon_bit;
        if(lat_r.max - latitude >= latitude - lat_r.min) {
            lat_bit = 0;
            lat_r.max = (lat_r.max + lat_r.min) / 2;
        } else {
            lat_bit = 1;
            lat_r.min = (lat_r.max + lat_r.min) / 2;
        }
        if(lon_r.max - longitude >= longitude - lon_r.min) {
            lon_bit = 0;
            lon_r.max = (lon_r.max + lon_r.min) / 2;
        } else {
            lon_bit = 1;
            lon_r.min = (lon_r.max + lon_r.min) / 2;
        }
        hash.bits <<= 1;
        hash.bits += lon_bit;
        hash.bits <<= 1;
        hash.bits += lat_bit;
    }

    return hash;
}

GeoHashArea geohash_decode(GeoHashRange lat_range, GeoHashRange lon_range,
                 GeoHashBits hash) {

    var area = GeoHashArea(hash, lat_range.clone(), lon_range.clone());
    for(int i = 0; i < hash.step; i++) {
        int lat_bit, lon_bit;
        lon_bit = _get_bit(hash.bits, (hash.step - i) * 2 - 1);
        lat_bit = _get_bit(hash.bits, (hash.step - i) * 2 - 2);
        if(lat_bit == 0)
            area.latitude.max = (area.latitude.max + area.latitude.min) / 2;
        else
            area.latitude.min = (area.latitude.max + area.latitude.min) / 2;
        if(lon_bit == 0)
            area.longitude.max = (area.longitude.max + area.longitude.min) / 2;
        else
            area.longitude.min = (area.longitude.max + area.longitude.min) / 2;
    }
    return area;
}

// Fast implementation
// ==================================================
final _B = Uint64List.fromList([
    0x5555555555555555, 0x3333333333333333, 0x0F0F0F0F0F0F0F0F,
    0x00FF00FF00FF00FF, 0x0000FFFF0000FFFF, 0x00000000FFFFFFFF
]);
final _S = Int8List.fromList([0,1,2,4,8,16]);

// Interleave lower  bits of x and y, so the bits of x
// are in the even positions and bits from y in the odd;
// https://graphics.stanford.edu/~seander/bithacks.html#InterleaveBMN

int _interleave64(int x, int y) {
    x = (x | (x << _S[5])) & _B[4];
    y = (y | (y << _S[5])) & _B[4];

    x = (x | (x << _S[4])) & _B[3];
    y = (y | (y << _S[4])) & _B[3];

    x = (x | (x << _S[3])) & _B[2];
    y = (y | (y << _S[3])) & _B[2];

    x = (x | (x << _S[2])) & _B[1];
    y = (y | (y << _S[2])) & _B[1];

    x = (x | (x << _S[1])) & _B[0];
    y = (y | (y << _S[1])) & _B[0];

    return x | (y << 1);
}

///reverse the interleave process
// http://stackoverflow.com/questions/4909263/how-to-efficiently-de-interleave-bits-inverse-morton
int _deinterleave64(int i) {
    int x = i;
    int y = i >> 1;

    x = (x | (x >> _S[0])) & _B[0];
    y = (y | (y >> _S[0])) & _B[0];

    x = (x | (x >> _S[1])) & _B[1];
    y = (y | (y >> _S[1])) & _B[1];

    x = (x | (x >> _S[2])) & _B[2];
    y = (y | (y >> _S[2])) & _B[2];

    x = (x | (x >> _S[3])) & _B[3];
    y = (y | (y >> _S[3])) & _B[3];

    x = (x | (x >> _S[4])) & _B[4];
    y = (y | (y >> _S[4])) & _B[4];

    x = (x | (x >> _S[5])) & _B[5];
    y = (y | (y >> _S[5])) & _B[5];

    return x | (y << 32);
}

// Return null on failure
GeoHashBits geohash_fast_encode(GeoHashRange lat_range, GeoHashRange lon_range,
        double latitude, double longitude, int step)
{
    if(step > 32 || step <= 0)
        return null;

    if(latitude < lat_range.min || latitude > lat_range.max ||
       longitude < lon_range.min || longitude > lon_range.max)
       return null;

    var hash = GeoHashBits(0, step);

    // The algorithm computes the morton code for the geohash location within
    // the range this can be done MUCH more efficiently using the following code

    //compute the coordinate in the range 0-1
    double lat_offset = (latitude - lat_range.min) / (lat_range.max - lat_range.min);
    double lon_offset = (longitude - lon_range.min) / (lon_range.max - lon_range.min);
    //convert it to fixed point based on the step size
    lat_offset *= (1 << step);
    lon_offset *= (1 << step);

    //interleave the bits to create the morton code.  No branching and no bounding
    hash.bits = _interleave64(lat_offset.toInt() & 0xFFFFFFFF,
                              lon_offset.toInt() & 0xFFFFFFFF);
    return hash;
}

GeoHashArea geohash_fast_decode(GeoHashRange lat_range, GeoHashRange lon_range,
                 GeoHashBits hash) {

    int step = hash.step;
    int xyhilo = _deinterleave64(hash.bits);

    double lat_scale = lat_range.max - lat_range.min;
    double lon_scale = lon_range.max - lon_range.min;

    int ilato = xyhilo & 0xFFFFFFFF;
    int ilono = (xyhilo >> 32) & 0xFFFFFFFF;
    return GeoHashArea(
        hash.clone(),
        GeoHashRange(
            lat_range.min + (ilato * 1.0 / (0x01 << step)) * lat_scale,
            lat_range.min + ((ilato + 1) * 1.0 / (0x01 << step)) * lat_scale
        ),
        GeoHashRange(
            lon_range.min + (ilono * 1.0 / (0x01 << step)) * lon_scale,
            lon_range.min + ((ilono + 1) * 1.0 / (0x01 << step)) * lon_scale
        )
    );
}

// ================== End of fast implementation ============

// Utilities

int _get_bit(int bits, int pos) {
    return (bits >> pos) & 0x01;
}

GeoHashBits _geohash_move_x(GeoHashBits hash, int d) {
    if(d == 0)
        return hash;
    int x = hash.bits & 0xaaaaaaaaaaaaaaaa;
    int y = hash.bits & 0x5555555555555555;
    int zz = 0x5555555555555555 >> (hash.step * 2);
    if(d > 0)
        x = x + (zz + 1);
    else {
        x |= zz;
        x = x - (zz + 1);
    }
    x &= (0xaaaaaaaaaaaaaaaa >> (64 - hash.step *2));
    hash.bits = x | y;
    return hash;
}

GeoHashBits _geohash_move_y(GeoHashBits hash, int d) {
    if(d == 0)
        return hash;
    int x = hash.bits & 0xaaaaaaaaaaaaaaaa;
    int y = hash.bits & 0x5555555555555555;
    int zz = 0xaaaaaaaaaaaaaaaa >> (64 - hash.step * 2);
    if(d > 0)
        y = y + (zz + 1);
    else {
        y |= zz;
        y = y - (zz + 1);
    }
    y &= (0x5555555555555555 >> (64 - hash.step *2));
    hash.bits = x | y;
    return hash;
}

GeoHashNeighbors geohash_get_neighbors(GeoHashBits hash) {
    return GeoHashNeighbors(
        north: geohash_get_neighbor(hash, GeoDirection.GEOHASH_NORTH),
        east: geohash_get_neighbor(hash, GeoDirection.GEOHASH_EAST),
        west: geohash_get_neighbor(hash, GeoDirection.GEOHASH_WEST),
        south: geohash_get_neighbor(hash, GeoDirection.GEOHASH_SOUTH),
        south_west: geohash_get_neighbor(hash, GeoDirection.GEOHASH_SOUTH_WEST),
        south_east: geohash_get_neighbor(hash, GeoDirection.GEOHASH_SOUTH_WEST),
        north_east: geohash_get_neighbor(hash, GeoDirection.GEOHASH_NORTH_EAST),
        north_west: geohash_get_neighbor(hash, GeoDirection.GEOHASH_NORTH_WEST),
    );
}

GeoHashBits geohash_get_neighbor(GeoHashBits hash, GeoDirection direction) {
    int x, y;
    switch(direction) {
        case GeoDirection.GEOHASH_NORTH:
            x = 0; y = 1; break;
        case GeoDirection.GEOHASH_EAST:
            x = 1; y = 0; break;
        case GeoDirection.GEOHASH_WEST:
            x = -1; y = 0; break;
        case GeoDirection.GEOHASH_SOUTH:
            x = 0; y = -1; break;
        case GeoDirection.GEOHASH_SOUTH_WEST:
            x = -1; y = -1; break;
        case GeoDirection.GEOHASH_SOUTH_EAST:
            x = 1; y = -1; break;
        case GeoDirection.GEOHASH_NORTH_WEST:
            x = -1; y = 1; break;
        case GeoDirection.GEOHASH_NORTH_EAST:
            x = 1; y = 1; break;
    }
    return _geohash_move_y(_geohash_move_x(hash.clone(), x).clone(), y);
}

GeoHashBits geohash_next_leftbottom(GeoHashBits bits) =>
    GeoHashBits(bits.bits << 2, bits.step + 1);

GeoHashBits geohash_next_rightbottom(GeoHashBits bits) =>
    GeoHashBits((bits.bits << 2) + 2, bits.step + 1);

GeoHashBits geohash_next_lefttop(GeoHashBits bits) =>
    GeoHashBits((bits.bits << 2) + 1, bits.step + 1);

GeoHashBits geohash_next_righttop(GeoHashBits bits) =>
    GeoHashBits((bits.bits << 2) + 3, bits.step + 1);

// Validates that coordinates are within range we can handle
// throws ArgumentError otherwise
void validateCoords(double lat, double lon) {
    if(lat == null)
        throw ArgumentError.notNull("latitude");
    if(lon == null)
        throw ArgumentError.notNull("longitude");
    if(lat < GeoHashRange.WGS84_LAT_MIN ||
       lat > GeoHashRange.WGS84_LAT_MAX)
       throw ArgumentError.value(
            lat, "latitude",
            "should be within range "
            "${GeoHashRange.WGS84_LAT_MIN}-${GeoHashRange.WGS84_LAT_MAX}"
       );
    if(lon < GeoHashRange.WGS84_LON_MIN ||
       lon > GeoHashRange.WGS84_LON_MAX)
       throw ArgumentError.value(
            lon, "longitude",
            "should be within range "
            "${GeoHashRange.WGS84_LON_MIN}-${GeoHashRange.WGS84_LON_MAX}"
       );

}

double _degreesToRadians(double deg) => (deg * Math.pi)/180.0;

// Adopted from https://github.com/firebase/geofire-js
// MIT License
// Returns distance in meters between two points using
// haversine formula (assuming Earth is a sphere)
double haversineDistance(double lat1, double lon1, double lat2, double lon2) {
    validateCoords(lat1, lon1);
    validateCoords(lat2, lon2);

    const radius = 6371000; // Earth's radius
    var latDelta = _degreesToRadians(lat2 - lat1);
    var lonDelta = _degreesToRadians(lon2 - lon1);

    var a = (Math.sin(latDelta / 2) * Math.sin(latDelta / 2)) +
    (Math.cos(_degreesToRadians(lat1)) * Math.cos(_degreesToRadians(lat2)) *
     Math.sin(lonDelta / 2) * Math.sin(lonDelta / 2));

    var c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));

    return radius * c;
}


class _RadiusBits {
    final double radius; // Search radius in meters
    final int    bits;   // Bits of geohash precision
    static const _radiusBits = <_RadiusBits>[
        _RadiusBits(52, 0.5971),
        _RadiusBits(50, 1.1943),
        _RadiusBits(48, 2.3889),
        _RadiusBits(46, 4.7774),
        _RadiusBits(44, 9.5547),
        _RadiusBits(42, 19.1095),
        _RadiusBits(40, 38.2189),
        _RadiusBits(38, 76.4378),
        _RadiusBits(36, 152.8757),
        _RadiusBits(34, 305.751),
        _RadiusBits(32, 611.5028),
        _RadiusBits(30, 1223.0056),
        _RadiusBits(28, 2446.0112),
        _RadiusBits(26, 4892.0224),
        _RadiusBits(24, 9784.0449),
        _RadiusBits(22, 19568.0898),
        _RadiusBits(20, 39136.1797),
        _RadiusBits(18, 78272.35938),
        _RadiusBits(16, 156544.7188),
        _RadiusBits(14, 313089.4375),
        _RadiusBits(12, 626178.875),
        _RadiusBits(10, 1252357.75),
        _RadiusBits(8, 2504715.5),
        _RadiusBits(6, 5009431),
        _RadiusBits(4, 10018863),
    ];
    const _RadiusBits(this.bits, this.radius);

    // Finds appropriate bit width for geohash for given search radius in meters
    static int findStepByRadius(double radius) {
        if(radius < 0.5971 || radius > 10018863)
            throw ArgumentError.value(radius, "radius",
                "should be in range 0.5971-10018863m" );
        // binary search, very clever
        int min = 0;
        int max = _RadiusBits._radiusBits.length;
        int mid;
        while(min < max) {
            mid = min + ((max - min) >> 1);
            final e = _RadiusBits._radiusBits[mid];
            final diff = e.radius - radius;
            if(diff.abs() < 0.00001) // this is our max precision in the table
                return e.bits;       // we found exact radius
            if(diff < 0.0)
                min = mid + 1;
            else
                max = mid;
        }
        // If not exact match return bits for the next biggest radius
        return _RadiusBits._radiusBits[min].bits;
    }

}

TreeSet<GeoHashBits>
geohash_get_areas_by_radius(double lat, double lon, double radius) {
    final results = TreeSet<GeoHashBits>(
        comparator: (a, b) {
            if(a.step < b.step)
                return -1;
            if(a.step > b.step)
                return 1;
            if(a.bits < b.bits)
                return -1;
            if(a.bits > b.bits)
                return 1;
            return 0;
        }
    );
    final lat_range = GeoHashRange.getStandardRange(
        GeoProjection.WGS84, GeoCoordinate.Latitude
    );
    final lon_range = GeoHashRange.getStandardRange(
        GeoProjection.WGS84, GeoCoordinate.Longitude
    );
    final double delta_lon = radius / (111320.0 * Math.cos(_degreesToRadians(lat)));
    final double delta_lat = radius / 110540.0;
    final double max_lat = lat + delta_lat;
    final double min_lat = lat - delta_lat;
    final double max_lon = lon + delta_lon;
    final double min_lon = lon - delta_lon;
    final int steps = _RadiusBits.findStepByRadius(radius) >> 1;
    final hash = geohash_fast_encode(lat_range, lon_range, lat, lon, steps);
    final area = geohash_fast_decode(lat_range, lon_range, hash);
    assert(area.within(lat, lon));

    results.add(hash);
    final double range_lon = (area.longitude.max - area.longitude.min) / 2;
    final double range_lat = (area.latitude.max - area.latitude.min) / 2;
    bool split_east = false;
    bool split_west = false;
    bool split_north = false;
    bool split_south = false;
    GeoHashNeighbors hood = GeoHashNeighbors();

    if(max_lon > area.longitude.max) {
        hood.east = geohash_get_neighbor(hash, GeoDirection.GEOHASH_EAST);
        if(area.longitude.max + range_lon > max_lon) {
            results.add(geohash_next_leftbottom(hood.east));
            results.add(geohash_next_lefttop(hood.east));
            split_east = true;
        } else {
            results.add(hood.east);
        }
    }
    if(min_lon < area.longitude.min){
        hood.west = geohash_get_neighbor(hash, GeoDirection.GEOHASH_WEST);
        if(area.longitude.min - range_lon < min_lon) {
            results.add(geohash_next_rightbottom(hood.west));
            results.add(geohash_next_righttop(hood.west));
            split_west = true;
        } else {
            results.add(hood.west);
        }
    }
    if(max_lat > area.latitude.max) {
        hood.north = geohash_get_neighbor(hash, GeoDirection.GEOHASH_NORTH);
        if(area.latitude.max + range_lat > max_lat) {
            results.add(geohash_next_rightbottom(hood.north));
            results.add(geohash_next_leftbottom(hood.north));
            split_north = true;
        } else {
            results.add(hood.north);
        }
    }
    if(min_lat < area.latitude.min) {
        hood.south = geohash_get_neighbor(hash, GeoDirection.GEOHASH_SOUTH);
        if(area.latitude.min - range_lat < min_lat) {
            results.add(geohash_next_righttop(hood.south));
            results.add(geohash_next_lefttop(hood.south));
            split_south = true;
        } else {
            results.add(hood.south);
        }
    }
    if(max_lon > area.longitude.max && max_lat > area.latitude.max) {
        hood.north_east = geohash_get_neighbor(hash, GeoDirection.GEOHASH_NORTH_EAST);
        if(split_north && split_east) {
            results.add(geohash_next_leftbottom(hood.north_east));
        }
        else if (split_north)
        {
            results.add(geohash_next_rightbottom(hood.north_east));
            results.add(geohash_next_leftbottom(hood.north_east));
        }
        else if (split_east)
        {
            results.add(geohash_next_leftbottom(hood.north_east));
            results.add(geohash_next_lefttop(hood.north_east));
        }
        else
        {
            results.add(hood.north_east);
        }
    }
    if(max_lon > area.longitude.max && min_lat < area.latitude.min) {
        hood.south_east = geohash_get_neighbor(hash, GeoDirection.GEOHASH_SOUTH_EAST);
        if(split_south && split_east) {
            results.add(geohash_next_lefttop(hood.south_east));
        } else if(split_south) {
            results.add(geohash_next_righttop(hood.south_east));
            results.add(geohash_next_lefttop(hood.south_east));
        } else if(split_east) {
            results.add(geohash_next_leftbottom(hood.south_east));
            results.add(geohash_next_lefttop(hood.south_east));
        } else {
            results.add(hood.south_east);
        }
    }
    if(min_lon < area.longitude.min && max_lat > area.latitude.max) {
        hood.north_west = geohash_get_neighbor(hash, GeoDirection.GEOHASH_NORTH_WEST);
        if(split_north && split_west) {
            results.add(geohash_next_rightbottom(hood.north_west));
        } else if(split_north) {
            results.add(geohash_next_rightbottom(hood.north_west));
            results.add(geohash_next_leftbottom(hood.north_west));
        } else if(split_west) {
            results.add(geohash_next_rightbottom(hood.north_west));
            results.add(geohash_next_righttop(hood.north_west));
        } else {
            results.add(hood.north_west);
        }
    }
    if(min_lon < area.longitude.min && min_lat < area.latitude.min) {
        hood.south_west = geohash_get_neighbor(hash, GeoDirection.GEOHASH_SOUTH_WEST);
        if(split_south && split_west) {
            results.add(geohash_next_righttop(hood.south_west));
        } else if(split_south) {
            results.add(geohash_next_righttop(hood.south_west));
            results.add(geohash_next_lefttop(hood.south_west));
        } else if(split_west) {
            results.add(geohash_next_rightbottom(hood.south_west));
            results.add(geohash_next_righttop(hood.south_west));
        } else {
            results.add(hood.south_west);
        }
    }
    return results;
}

