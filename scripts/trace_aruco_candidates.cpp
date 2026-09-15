// Diagnostic only. Uses version-pinned OpenCV functions prepared in build/.
#include "trace_functions.hpp"
#include <filesystem>
#include <iostream>
#include <set>

void quad(FileStorage &fs, const vector<Point2f>& q) {
    fs << "[";
    for(auto p:q) fs << "[" << p.x << p.y << "]";
    fs << "]";
}
int identify(const Mat& img, const vector<Point2f>& q, Ptr<Dictionary> dict,
             Ptr<DetectorParameters> params) {
    int id=-1, rotation=0;
    return _identifyOneCandidate(dict,img,q,id,params,rotation) ? id : -1;
}
int main(int argc,char**argv) {
    if(argc!=3) return 2;
    if(std::filesystem::exists(argv[2])) return 3;
    const string root=argv[1];
    FileStorage fs(argv[2], FileStorage::WRITE|FileStorage::FORMAT_JSON);
    fs << "opencv" << CV_VERSION << "traces" << "[";
    auto dict=getPredefinedDictionary(DICT_4X4_50);
    int total=0, matched=0;
    for(string mode:{"raw_route","face_compare_pbr","face_compare_cells"}) {
      for(int target:{0,20,30,45}) for(int distance:{300,200,150,120,100,75}) {
        string name="id"+to_string(target)+"_"+to_string(distance)+"cm";
        Mat gray=imread(root+"/"+mode+"/"+name+".png",IMREAD_GRAYSCALE);
        if(gray.empty()) return 4;
        for(double rate:{.05,.02}) {
          auto params=DetectorParameters::create();
          params->minMarkerDistanceRate=rate;
          // Same initialization as detectMarkers when ArUco3 is disabled.
          params->minSideLengthCanonicalImg=0;
          params->minMarkerLengthRatioOriginalImg=0;
          vector<vector<Point2f>> candidates; vector<vector<Point>> contours;
          _detectInitialCandidates(gray,candidates,contours,params);
          _reorderCandidatesCorners(candidates);
          vector<int> windows;
          for(int win=3;win<=23;win+=10) {
            Mat bw; _threshold(gray,bw,win,7);
            vector<vector<Point2f>> q; vector<vector<Point>> c;
            _findMarkerContours(bw,q,c,.03,4,.03,.05,3,0);
            _reorderCandidatesCorners(q);
            for(size_t k=0;k<q.size();k++) {
              if(q[k]!=candidates[windows.size()]) return 5;
              windows.push_back(win);
            }
          }
          vector<vector<vector<Point2f>>> grouped; vector<vector<vector<Point>>> gc;
          _filterTooCloseCandidates(candidates,grouped,contours,gc,rate,false);
          vector<int> trace_ids,builtin_ids;
          for(auto &q:grouped[0]) { int id=identify(gray,q,dict,params); if(id>=0)trace_ids.push_back(id); }
          vector<vector<Point2f>> bc,br;
          detectMarkers(gray,dict,bc,builtin_ids,params,br);
          sort(trace_ids.begin(),trace_ids.end()); sort(builtin_ids.begin(),builtin_ids.end());
          total++; matched+=trace_ids==builtin_ids;
          fs << "{" << "case" << mode << "view" << name << "target" << target
             << "rate" << rate << "builtin_ids" << builtin_ids << "trace_ids" << trace_ids
             << "match" << (trace_ids==builtin_ids) << "candidates" << "[";
          for(size_t i=0;i<candidates.size();i++) {
            Mat bits=_extractBits(gray,candidates[i],4,1,4,.13,5);
            fs << "{" << "index" << int(i) << "window" << windows[i]
               << "contour_length" << int(contours[i].size())
               << "area" << contourArea(candidates[i])
               << "standalone_id" << identify(gray,candidates[i],dict,params)
               << "border_errors" << _getBorderErrors(bits,4,1)
               << "border_error_limit" << 5 << "corners";
            quad(fs,candidates[i]); fs << "}";
          }
          fs << "]" << "groups" << "[";
          for(size_t g=0;g<trace_groups.size();g++) {
            auto &q=grouped[0][g]; int kept=-1;
            for(auto i:trace_groups[g]) if(candidates[i]==q)kept=int(i);
            vector<int> members(trace_groups[g].begin(),trace_groups[g].end());
            fs << "{" << "members" << members << "kept" << kept
               << "decoded_id" << identify(gray,q,dict,params) << "}";
          }
          fs << "]" << "}";
        }
      }
    }
    fs << "]" << "comparisons" << total << "builtin_matches" << matched;
    cout<<"comparisons="<<total<<" builtin_matches="<<matched<<endl;
    return matched==total?0:1;
}
