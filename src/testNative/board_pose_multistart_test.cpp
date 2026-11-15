#define DJI_BOARD_POSE_REPLAY_TEST 1
#include "../main/cpp/dji_board_vision.cpp"
#include <fstream>
#include <iostream>
#include <numeric>
#include <stdexcept>
#include <ctime>
#include <cstdlib>
#include <atomic>

static std::atomic<size_t> allocationCount{0};
void* operator new(size_t size) { allocationCount.fetch_add(1, std::memory_order_relaxed); if (void* p=std::malloc(size ? size : 1)) return p; throw std::bad_alloc(); }
void operator delete(void* p) noexcept { std::free(p); }
void operator delete(void* p, size_t) noexcept { std::free(p); }
void* operator new[](size_t n) { return ::operator new(n); }
void operator delete[](void* p) noexcept { ::operator delete(p); }

struct Fixture {
    std::string session; int64_t id; int tags, originalId; bool promoted;
    double fx,fy,cx,cy,loggedRms,expectedRms; Distortion distortion{};
    RigidPose initial{}, logged{}; Quaternion expectedQ{};
    std::vector<Correspondence> observations;
    std::vector<VisibleMarker> visible;
    std::vector<IndependentMarkerPose> independent;
};
struct Result { bool valid; RigidPose pose; double rms=0,maximum=0; std::string reason; MultiStartDiagnostics diagnostics; };
static int passed=0,failed=0;
static std::ofstream xml;
static void Check(bool value,const std::string& name) {
    xml << "<test-case name=\"" << name << "\" result=\"" << (value?"Passed":"Failed") << "\"/>\n";
    if(value)++passed;else {++failed;std::cerr << "FAIL " << name << '\n';}
}
static RigidPose ReadPose(std::istream& in) { RigidPose p;in>>p.q.x>>p.q.y>>p.q.z>>p.q.w>>p.t.x>>p.t.y>>p.t.z;return p; }
static std::vector<Fixture> Load(const char* path) {
    std::ifstream in(path);size_t n;in>>n;if(!in)throw std::runtime_error("Missing fixtures");
    std::vector<Fixture> result;
    for(size_t i=0;i<n;++i) {
        Fixture f;size_t corners,seeds;int promoted;
        in>>f.session>>f.id>>f.tags>>f.originalId>>corners>>seeds>>promoted;f.promoted=promoted!=0;
        in>>f.fx>>f.fy>>f.cx>>f.cy>>f.distortion.k1>>f.distortion.k2>>f.distortion.p1>>f.distortion.p2>>f.distortion.k3;
        f.initial=ReadPose(in);f.logged=ReadPose(in);
        in>>f.loggedRms>>f.expectedRms>>f.expectedQ.x>>f.expectedQ.y>>f.expectedQ.z>>f.expectedQ.w;
        for(size_t j=0;j<corners;++j) {
            Correspondence c;in>>c.markerId>>c.cornerIndex>>c.boardPoint.x>>c.boardPoint.y>>c.boardPoint.z>>c.imagePoint.x>>c.imagePoint.y;
            f.observations.push_back(c);
            if(std::none_of(f.visible.begin(),f.visible.end(),[&](const VisibleMarker& v){return v.id==c.markerId;})) {VisibleMarker v{};v.id=c.markerId;f.visible.push_back(v);}
        }
        for(size_t j=0;j<seeds;++j) {IndependentMarkerPose p{};in>>p.id;p.cameraFromBoard=ReadPose(in);f.independent.push_back(p);}
        if(!in)throw std::runtime_error("Truncated fixture");result.push_back(f);
    }
    return result;
}
static Result Run(const Fixture& f,bool enabled,bool available=true) {
    Result r;r.pose=f.initial;
    r.valid=RefineBoardPoseSelection(enabled,f.observations,f.visible,f.independent,f.originalId,available,f.initial,
        f.fx,f.fy,f.cx,f.cy,f.distortion,&r.pose,&r.rms,&r.maximum,&r.reason,false,&r.diagnostics);
    return r;
}
static void Synthetic(const Fixture& base) {
    auto f=base;auto original=Run(f,false);
    auto runInjected=[&](auto refine){Result r;r.pose=f.initial;r.valid=SelectFourTagPoseHypotheses(f.observations,f.visible,f.independent,
        f.originalId,true,f.initial,f.fx,f.fy,f.cx,f.cy,f.distortion,&r.pose,&r.rms,&r.maximum,&r.reason,&r.diagnostics,refine);return r;};
    int calls=0;
    auto guarded=runInjected([&](RigidPose* p,double* rms,double* maximum,std::string*){
        if(++calls>1)return false;*p=original.pose;*rms=original.rms;*maximum=original.maximum;return true;});
    Check(guarded.valid&&guarded.diagnostics.hypotheses[guarded.diagnostics.selected].original,"invalid_extra_preserves_valid_original");
    calls=0;
    auto throwing=runInjected([&](RigidPose* p,double* rms,double* maximum,std::string*){
        if(++calls>1)throw std::runtime_error("test");*p=original.pose;*rms=original.rms;*maximum=original.maximum;return true;});
    Check(throwing.valid,"extra_exception_preserves_original");
    auto invalid=runInjected([](RigidPose* p,double* rms,double* maximum,std::string*){
        p->t.z=-1;*rms=0;*maximum=0;return true;});
    Check(!invalid.valid,"negative_depth_not_selected");
    auto nan=runInjected([](RigidPose* p,double* rms,double* maximum,std::string*){
        p->q.x=std::numeric_limits<double>::quiet_NaN();*rms=0;*maximum=0;return true;});
    Check(!nan.valid,"nonfinite_refined_pose_not_selected");
    auto overflow=runInjected([](RigidPose* p,double* rms,double* maximum,std::string*){
        p->t={1e100,1e100,1e100};*rms=0;*maximum=0;return true;});
    Check(!overflow.valid,"jni_float_overflow_not_selected");
    auto no=runInjected([](RigidPose*,double*,double*,std::string*){return false;});
    Check(!no.valid&&no.reason=="MULTISTART_NO_VALID_REFINED_POSE","all_failed_preserves_failure_semantics");
    f.initial.q={0,0,0,0};auto zero=Run(f,true);
    Check(zero.valid&&!zero.diagnostics.hypotheses[0].valid,"zero_quaternion_is_invalid_other_seeds_survive");
    f=base;f.independent.resize(1);auto fewer=Run(f,true);
    Check(fewer.valid,"fewer_usable_initializers_original_survives");
    f=base;auto missing=Run(f,true,false);Check(missing.valid,"missing_original_uses_remaining_initializers");
    f=base;f.independent.clear();Check(!Run(f,true,false).valid,"no_available_initializer_is_failure");
    f=base;f.visible[1].id=f.visible[0].id;Check(!Run(f,true).valid,"duplicate_detected_ids_rejected");
    f=base;f.observations[1].cornerIndex=0;Check(!Run(f,true).valid,"duplicate_or_missing_corner_rejected");
    f=base;f.independent.push_back(f.independent.back());Check(Run(f,true).valid,"duplicate_seed_id_does_not_erase_original");
    MultiStartHypothesis a,b;a.original=true;a.markerId=3;a.finalRms=1.0;b.markerId=0;b.finalRms=1.0-1e-9;
    Check(MultiStartBetter(a,b)&&!MultiStartBetter(b,a),"same_published_float_prefers_original");
    b.finalRms=.999;Check(MultiStartBetter(b,a),"materially_lower_rms_wins");
    a.original=false;a.markerId=1;b.markerId=0;b.finalRms=a.finalRms;
    Check(MultiStartBetter(b,a),"nonoriginal_ties_use_stable_tag_id");
    f=base;for(auto& p:f.independent)p.cameraFromBoard.q={-p.cameraFromBoard.q.x,-p.cameraFromBoard.q.y,-p.cameraFromBoard.q.z,-p.cameraFromBoard.q.w};
    f.initial.q={-f.initial.q.x,-f.initial.q.y,-f.initial.q.z,-f.initial.q.w};
    const auto signedPose=Run(f,true),normal=Run(base,true);
    Check(signedPose.valid&&QuaternionAngularDifferenceDegrees(signedPose.pose.q,normal.pose.q)<.002,"q_and_negative_q_equivalent");
    for(int count=1;count<=3;++count){f=base;f.visible.resize(count);
        f.observations.erase(std::remove_if(f.observations.begin(),f.observations.end(),[&](const Correspondence& c){
            return std::none_of(f.visible.begin(),f.visible.end(),[&](const VisibleMarker& v){return v.id==c.markerId;});}),f.observations.end());
        auto x=Run(f,false),y=Run(f,true);Check(x.valid==y.valid&&x.rms==y.rms&&QuaternionAngularDifferenceDegrees(x.pose.q,y.pose.q)<1e-5,"lower_tag_path_unchanged_"+std::to_string(count));}
}
int main(int argc,char** argv) {
    if(argc<3){std::cerr<<"fixture_path output_prefix [benchmark_repeats]\n";return 2;}
    try {
        const auto fixtures=Load(argv[1]);const std::string prefix=argv[2];
        xml.open(prefix+"-cases.xml");xml<<"<test-suite name=\"NativeFourTagMultiStart\">\n";
        std::ofstream csv(prefix+"-results.csv"),diagnostics(prefix+"-diagnostics.jsonl");
        csv<<"session,id,tags,promoted,selectedId,legacyRms,selectedRms,angleFromLegacyDeg,qx,qy,qz,qw,tx,ty,tz\n";
        csv<<std::setprecision(17);
        for(const auto& f:fixtures){auto legacy=Run(f,false),selected=Run(f,true),compiled=Run(f,DJI_APRILTAG_FOUR_TAG_MULTISTART_EXPERIMENT!=0);
            const auto name=std::to_string(f.id);Check(legacy.valid,name+"_legacy_valid");
            Check(std::abs(legacy.rms-f.loggedRms)<.0003&&QuaternionAngularDifferenceDegrees(legacy.pose.q,f.logged.q)<.002,name+"_recorded_native_parity");
            Check(selected.valid&&std::abs(selected.rms-f.expectedRms)<.0003,name+"_python_expected_rms");
            Check(selected.valid&&QuaternionAngularDifferenceDegrees(selected.pose.q,f.expectedQ)<.002,name+"_python_expected_orientation");
            Check(selected.rms<=legacy.rms+std::numeric_limits<float>::epsilon()*std::max(1.,legacy.rms),name+"_no_worse_published_rms");
            auto stats=EvaluateReprojection(selected.pose,f.observations,f.fx,f.fy,f.cx,f.cy,f.distortion,false);
            Check(stats.valid&&IsFinite(selected.pose.q)&&IsFinite(selected.pose.t),name+"_valid_depth_finite");
            Check(DJI_APRILTAG_FOUR_TAG_MULTISTART_EXPERIMENT ? compiled.rms==selected.rms : compiled.rms==legacy.rms,name+"_compile_switch_routes_correctly");
            if(f.tags==4){Check(selected.diagnostics.count>0&&selected.diagnostics.hypotheses[0].original&&selected.diagnostics.hypotheses[0].markerId==f.originalId,name+"_original_evaluated");
                auto reordered=f;std::reverse(reordered.visible.begin(),reordered.visible.end());std::reverse(reordered.independent.begin(),reordered.independent.end());
                auto reversed=Run(reordered,true);Check(reversed.valid&&static_cast<float>(reversed.rms)==static_cast<float>(selected.rms),name+"_order_independent_rms");
                for(const auto& seed:f.independent){auto changed=f;changed.originalId=seed.id;changed.initial=seed.cameraFromBoard;
                    auto reorderedOriginal=Run(changed,true);
                    Check(reorderedOriginal.valid&&std::abs(reorderedOriginal.rms-selected.rms)<=std::numeric_limits<float>::epsilon()*std::max(1.,selected.rms),
                        name+"_detector_first_id_"+std::to_string(seed.id));}}
            if(f.promoted&&f.session=="20261029T141917566Z")Check(QuaternionAngularDifferenceDegrees(selected.pose.q,f.logged.q)<.01,name+"_previously_promoted_preserved");
            if(f.id==125745904631969LL)Check(selected.diagnostics.hypotheses[selected.diagnostics.selected].markerId==3&&selected.rms<.415&&QuaternionAngularDifferenceDegrees(selected.pose.q,legacy.pose.q)>48,name+"_november_tag3_selected");
            const int id=selected.diagnostics.selected<0?f.originalId:selected.diagnostics.hypotheses[selected.diagnostics.selected].markerId;
            csv<<f.session<<','<<f.id<<','<<f.tags<<','<<f.promoted<<','<<id<<','<<legacy.rms<<','<<selected.rms<<','<<QuaternionAngularDifferenceDegrees(legacy.pose.q,selected.pose.q)<<','
                <<selected.pose.q.x<<','<<selected.pose.q.y<<','<<selected.pose.q.z<<','<<selected.pose.q.w<<','<<selected.pose.t.x<<','<<selected.pose.t.y<<','<<selected.pose.t.z<<'\n';
            auto json=FormatMultiStartDiagnostics(selected.diagnostics,f.visible,f.id,1,1);diagnostics<<json.data.data()<<'\n';
            Check(json.size<json.data.size()-1,name+"_bounded_diagnostic_json");
        }
        Synthetic(fixtures[1]);xml<<"</test-suite>\n";xml.close();
        const int repetitions=argc>3?std::atoi(argv[3]):10;
        std::ofstream bench(prefix+"-benchmark.csv");bench<<"mode,id,wallNs,refinementNs,allocations\n";
        const auto cpuStart=std::clock();
        for(int repeat=0;repeat<repetitions;++repeat)for(const auto& f:fixtures)if(f.tags==4)for(bool enabled:{false,true}){
            const size_t before=allocationCount.load();const auto began=std::chrono::steady_clock::now();auto r=Run(f,enabled);
            const auto ns=std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now()-began).count();
            const size_t allocations=allocationCount.load()-before;int64_t refine=0;for(size_t i=0;i<r.diagnostics.count;++i)refine+=r.diagnostics.hypotheses[i].refinementNs;
            bench<<(enabled?"multistart":"legacy")<<','<<f.id<<','<<ns<<','<<refine<<','<<allocations<<'\n';}
        std::ofstream summary(prefix+"-summary.json");summary<<"{\"passed\":"<<passed<<",\"failed\":"<<failed<<",\"compileExperiment\":"<<DJI_APRILTAG_FOUR_TAG_MULTISTART_EXPERIMENT<<",\"fixtures\":"<<fixtures.size()<<",\"benchmarkCpuSeconds\":"<<double(std::clock()-cpuStart)/CLOCKS_PER_SEC<<"}";
        std::cout<<"NATIVE_TESTS passed="<<passed<<" failed="<<failed<<" fixtures="<<fixtures.size()<<" compiledExperiment="<<DJI_APRILTAG_FOUR_TAG_MULTISTART_EXPERIMENT<<'\n';
        return failed?1:0;
    }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 2;}
}
