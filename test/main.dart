import 'dart:io';

import 'package:sprintf/sprintf.dart';
import 'package:benchmark_harness/benchmark_harness.dart';

import '../lib/geohash_int.dart';

class GHBenchParams {
    final lat_range = GeoHashRange(-20037726.37, 20037726.37);
    final lon_range = GeoHashRange(-20037726.37, 20037726.37);
    final double lat = 9741705.20;
    final double lon = 5417390.90;
}

class GHTestResults {
    static GeoHashBits hash_enc;
    static GeoHashBits hash_fast_enc;
    static GeoHashArea area_dec;
    static GeoHashArea area_fast_dec;
}

class GHBenchmarkEnc extends BenchmarkBase with GHBenchParams {
    GHBenchmarkEnc() : super("GeoHashEnc");

    void run() {
        GHTestResults.hash_enc = geohash_encode(lat_range, lon_range, lat, lon, 24);
    }

    void setup() {}
    void teardown() {}
}

class GHBenchmarkFEnc extends BenchmarkBase with GHBenchParams {
    GHBenchmarkFEnc() : super("GeoHashFastEnc");

    void run() {
        GHTestResults.hash_fast_enc = geohash_fast_encode(lat_range, lon_range, lat, lon, 24);
    }

    void setup() {}
    void teardown() {}
}

class GHBenchmarkDec extends BenchmarkBase with GHBenchParams {
    GHBenchmarkDec() : super("GeoHashDec");

    void run() {
        GHTestResults.area_dec = geohash_decode(lat_range, lon_range, GHTestResults.hash_enc);
    }

    void setup() {}
    void teardown() {}
}

class GHBenchmarkFDec extends BenchmarkBase with GHBenchParams {
    GHBenchmarkFDec() : super("GeoHashFastDec");

    void run() {
        GHTestResults.area_fast_dec = geohash_fast_decode(lat_range, lon_range, GHTestResults.hash_fast_enc);
    }

    void setup() {}
    void teardown() {}
}

void main() {
    GHBenchmarkEnc().report();
    GHBenchmarkFEnc().report();

    assert(GHTestResults.hash_enc == GHTestResults.hash_fast_enc);

    print("Hash: ${sprintf('%X',[GHTestResults.hash_enc.bits])}");
    print("Fast hash: ${sprintf('%X',[GHTestResults.hash_fast_enc.bits])}");

    GHBenchmarkDec().report();
    GHBenchmarkFDec().report();

    print("Area: ${GHTestResults.area_dec}");
    print("Fast area: ${GHTestResults.area_fast_dec}");

    assert(GHTestResults.area_dec == GHTestResults.area_fast_dec);
}

