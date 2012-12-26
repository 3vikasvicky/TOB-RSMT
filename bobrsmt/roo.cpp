#include "roo.h"
#include "BinaryHeap.h"

using namespace std;

/*-- global variables --*/
char error_text[BUFFERSIZE];
/*--------------  ERROR functions      ------------------*/
void runtimeError(char error_text[]) /* error handler */
{
    fprintf(stderr, "ERROR: %s \n", error_text);
    fprintf(stderr, "Aborting !! \n");
    fflush(stdout);
    fflush(stderr);
    exit(1);
}
/*---------------------------  Design functions   -----------------------------*/
void Design::readBlockFile (){
    
    char file_name[BUFFERSIZE], temp[BUFFERSIZE];
    strcpy(file_name, benchmarkPath);
    strcat(file_name, "/");
    strcat(file_name, blockFile);
    
    FILE *fp;
    if((fp=fopen(file_name, "r"))==NULL) {
        sprintf(error_text, "roo: Cannot open %s file", file_name);
        runtimeError(error_text);
    }
   

    char line[LINESIZE];
    *line = '\0';
    fgets(line, LINESIZE, fp);
    sscanf(line, "%s\t%*s\n", temp);
    sscanf(line, "DeltaX\t:\t%f\n", &deltaX);

    fgets(line, LINESIZE, fp);
    sscanf(line, "%s\t%*s\n", temp);
    sscanf(line, "Root\t:\t%d\n", &root);

    sinks_.clear();//read all sinks from pblock file
    int num_sinks;
    fscanf(fp, "%d\n", &num_sinks);
    for(int i=0;i<num_sinks;i++){
        Point sink;
        fscanf(fp,"%f",&sink.x);
        fscanf(fp,"%f",&sink.y);
        sinks_.push_back(sink);
    }
    int num_blocks;
    fscanf(fp, "%d\n", &num_blocks);
    for(int i=0;i<num_blocks;i++){
        fgets(line, LINESIZE, fp);
        char *pch = strtok (line, " \t");
        int x = 1;//indicating it's reading x coordinate
        Poly  pV;
        Point p;
        while (pch != NULL){
            if (x) {
                p.x = (atof(pch));
                xmin_ = min(xmin_, (float)p.x);
                xmax_ = max(xmax_, (float)p.x);
            }
            else  {
                p.y = (atof(pch));
                ymin_ = min(ymin_, (float)p.y);
                ymax_ = max(ymax_, (float)p.y);
                pV.push_back(p);
            }
            x = 1- x;
            pch = strtok (NULL, " \t");
        }
        blockVect.push_back(pV);
    }
}
void Design::reformBlock(){//This function keep joining overlapping edges in two adjacent blocks.
    int startA, startC, length;//startA is the index of A, startC is the index of C. startA+length is the index of B, startC+length is the index of D


    for (int i = 0; i < blockVect.size(); ++i){//size is changing by removing element
        Poly block1 = blockVect[i];
        bool flag = true; //flag is to indicate wether block1 is combined with any other block or not. If combined, the new shape could be adjacent to previous non-adjacent block
        while (flag){
            flag = false;
            for (int j = i+1; j < blockVect.size();++j){
                Poly block2 = blockVect[j];
                if (block1.IsAdjacent(block2)){
                    //block1.Plot();
                    //block2.Plot();
                    int index_pointA, index_pointC;
                    int num_adjacent_edges = block1.FindAdjacent(block2, &index_pointA, &index_pointC);
                    int index_pointB = (index_pointA + num_adjacent_edges) % block1.size();
                    int index_pointD = (index_pointC + num_adjacent_edges) % block2.size();
                    vector<Point> edgeAD, edgeCB;
                    edgeAD.push_back(block1.at(index_pointA));
                    if (block1.at(index_pointA) != block2.at(index_pointD)){//if AD(CB) is longer than 0
                        edgeAD.push_back(block2.at(index_pointD));
                    }
                    edgeCB.push_back(block2.at(index_pointC));
                    if (block2.at(index_pointC) != block1.at(index_pointB)){
                        edgeCB.push_back(block1.at(index_pointB));
                    }
                    Poly polyAD (edgeAD);
                    Poly polyCB (edgeCB);
                    Poly new_block1;
                    new_block1 = block1.cut(0, index_pointA-1) + polyAD;
                    if (index_pointD == 0)//index_pointD == 0 is special
                        new_block1 += block2.cut(index_pointD+1, index_pointC-1);
                    else
                        new_block1 += block2.cut(index_pointD+1, block2.size()-1) + block2.cut(0, index_pointC-1);
                    new_block1 += polyCB;
                    if (index_pointB != 0)
                        new_block1 += block1.cut(index_pointB+1, block1.size()-1);
                    blockVect[i] = new_block1;
                    block1 = blockVect[i];
                    blockVect.erase(blockVect.begin()+j);
                    --j;
                    flag = true;
                }
            }
        }
    }

}
void Design::AddPlacedLogics(){//This will randomly add new squares around existing blocks
    for (int i = 0; i < blockVect.size(); ++i){//size is changing by removing element
        Poly block1 = blockVect[i];
        Poly new_block1;
        for (int j = 0; j < block1.size(); ++j){//for each edge in block1
            Edge current_edge = block1.edge(j);
            Point p1= block1.p1_edge ((j+block1.size()-1)%block1.size());//p2 is the start of current edge and p3 is the end. p1 is the start point of previous edge. p1 is used to get the shape of new placed logics
            Point p2= block1.p1_edge (j);
            Point p3= block1.p2_edge (j);
            srand(time(0));
            float random_pos = rand()/(double)RAND_MAX;
            float random_height = rand()/(double)RAND_MAX;
            float random_length = rand()/(double)RAND_MAX;
            Point p4 = p2 + (p3-p2)*random_pos;//p4, p5, p6, p7
            Point p5 = p4 + (p2-p1)*random_height;
            Point p6 = p5 + (p3-p2)*random_length;
            Point p7 = p4 + p6 - p5;
            assert(p4!=p5 && p5!=p6 && p6!= p7);
            Poly random_block;
            random_block.push_back(p4);
            random_block.push_back(p5);
            random_block.push_back(p6);
            random_block.push_back(p7);
            new_block1.push_back(p2);
            if (!PointIsInBlocks(p5) && !PointIsInBlocks(p6)){// && !ContainSinkInBlock(random_block)){
                new_block1.push_back(p4);
                new_block1.push_back(p5);
                new_block1.push_back(p6);
                new_block1.push_back(p7);
            }
        }
        blockVect[i] = new_block1; 
    }
}

void Design::readBranchFile (){
    FILE *fp;
    char file_name[BUFFERSIZE], temp[BUFFERSIZE];
    char line[LINESIZE];
    char *pch = NULL;
    int degreeNum;
    int i, j, t, n;
    DTYPE x, y;
    set<int> index;
    vector<pair<int,float> > EP;//EP for tree is nodes
    set<Point>::iterator it;
    
    strcpy(file_name, benchmarkPath);
    strcat(file_name, "/");
    strcat(file_name, branchFile);
    
    if((fp=fopen(file_name, "r"))==NULL) {
        sprintf(error_text, "roo: Cannot open %s file", file_name);
        runtimeError(error_text);
    }
    
    while(!feof(fp)) {
        *line = '\0';
        fgets(line, LINESIZE, fp);
        sscanf(line, "%s\t%*s\n", temp);
        
        if( strlen(line)<5 || temp[0] == '#'||strcmp(temp,"DegreeNumber")==0){
            if( strcmp(temp,"DegreeNumber")==0)
                sscanf(line, "DegreeNumber\t:\t%d\n", temp, &degreeNum);
            continue;
        }
        sscanf(line, "%d\t%f\t%f%d\n", &t, &x, &y, &n);//t is index, n is the index of next branch
        assert (branchVect.size() == t);
        Branch b(branchVect.size(), Point(x,y), n);
        branchVect.push_back(b);
        xmin_ = min(xmin_, (float)x);
        xmax_ = max(xmax_, (float)x);
        ymin_ = min(ymin_, (float)y);
        ymax_ = max(ymax_, (float)y);
        for (i = 0; i < sinks_.size(); ++i){
            if (b.p==sinks_[i]){//generate EP for the tree
                pair<int,float> p(b.i, 0);
                EP.push_back(p);
            }
        }
    }
    for (i=0; i < branchVect.size(); i++){
        if (branchVect.at(i).p == branchVect.at(branchVect.at(i).n).p && i!=branchVect.at(i).n){//0 length branch remove, and there is one branch in the middle point to itself, leave this one alone
            int k = branchVect.at(i).n;
            for (j=0; j < branchVect.size(); j++){
                if (branchVect[j].n == i)   branchVect[j].n = k;
            }
            continue;
        }
        else{
            index.insert(i);
        }
    }
    tree = Tree (&branchVect, &blockVect, index, EP, root);//root can be anything, it was read in readBlockFile
}
void Design::sequenceBranch (int rt) {
    if (branchVect[rt].n == rt) return;//it's already sequenced
    initialSequenceBranch(rt);
}
void Design::initialSequenceBranch (int rt) {
    int current, previous, temp;
    current = rt;
    previous = rt;
    while (current!=previous || current==rt){//break when to the branch with n of itself. Just reverse all adverse branch
        temp = branchVect[current].n;
        branchVect[current].n = previous;
        previous = current;
        current = temp;
    }

}
void Design::splitBranch (){
    /*---------- From here is the intersection and contain part---------*/
    int iblock, ibranch;
    int i;
    Branch branch, newBranch, grandBranch;
    Point p;
    vector<Poly >::iterator iit;
//    vector<Branch>::size_type ibranch;
    set<int>::iterator it;
    set<int> index;
    Poly block;
    Edge brEdge, blEdge;
    set<int> sempty;

    for (iit = blockVect.begin(); iit != blockVect.end(); ++iit){
        iblock = iit-blockVect.begin();
        block = *iit;
        Tree_ tree_(&branchVect, &blockVect, -1, iblock, treeVect.size());

        set<int> roots;
        for (i = 0; i < block.size(); ++i){//intersection part
            blEdge = block.edge(i); 

            //            for (ibranch=0; ibranch < branchVect.size(); ++ibranch)//because branchVect will change size in the loop, we cannot use iterator!!
            index = tree.getIndex();//index will change in loop

            for (it=index.begin(); it !=  index.end(); it++){//because branchVect will change size in the loop, we cannot use iterator!!
                ibranch = *it;
                branch = branchVect.at(ibranch);
                brEdge = Edge (branch.p, branchVect[branch.n].p);
                if (blEdge.intersects(brEdge)) 
                {  
                    if (blEdge.v_) p = Point(blEdge.c_, brEdge.c_);//p is the crossing point of blEdge and brEdge
                    else p = Point (brEdge.c_, blEdge.c_);

                    newBranch = Branch(branchVect.size(), p, branch.n);
                    branchVect.push_back (newBranch);
                    branchVect.at(ibranch).n = newBranch.i;//assign to branch
                    branch = branchVect.at(ibranch);//refresh the branch
                    grandBranch = branchVect.at(newBranch.n);

                    if ( (block.contain(branch.p)||branch.p.IsOnPoly(block)) && grandBranch.p.isOutside(block) ){// branch is inside or on the edge(which is caused by two points are both outside the block), and the other end of branch is outside
                        if (branch.p.IsOnPoly(block) && tree.CountChildren(ibranch) == 0){
                            Branch zeroLengthBranch(branchVect.size(), branch.p, branch.i);
                            branchVect.push_back (zeroLengthBranch);
                            tree.insertIndex(zeroLengthBranch);
                        }
                        tree.eraseIndex(ibranch);
                        tree.insertIndex(newBranch);
                        tree_.insertIndex(branch);
                        tree_.setRoot(newBranch);//the i of branch will be the point of branch
                        roots.insert(newBranch.i);
                        if (branch.p.IsOnPoly(block))
                            tree_.insertEP(branch);
                    } else if (block.contain(grandBranch.p) || grandBranch.p.IsOnPoly(block)){//the other end of branch is inside block or on block
                        tree_.insertIndex(newBranch);
                        tree_.insertEP(newBranch);
                        if (grandBranch.p.IsOnPoly(block)){
                            tree_.setRoot(grandBranch);
                            roots.insert(grandBranch.i);
                        }
                    } else {//two points are both outside the block
                        tree.insertIndex(newBranch);
                    }
                }else if ( !branch.p.isOutside(block) && !branchVect.at(branch.n).p.isOutside(block) && tree.isIndex(ibranch) ){//branch inside block
                    if (branch.p.IsOnPoly(block) && branchVect.at(branch.n).p.IsOnPoly(block) && 
                            ((branch.p+branchVect.at(branch.n).p)/2).IsOnPoly(block))
                        continue;//branch is overlapping with one edge of one block, nothing should happen
                    tree.eraseIndex(ibranch);
                    tree_.insertIndex(branch);
                    if (branch.p.IsOnPoly(block))
                        tree_.insertEP(branch);
                    if (branchVect.at(branch.n).p.IsOnPoly(block)){
                        tree_.setRoot(branchVect.at(branch.n));
                        roots.insert(branch.n);
                    }
                }//end of intersect or not
            }//for one edge of block
        }//for one block
        //        tree.print(); verified
        if (roots.size()>1){//more than one tree in this block
            set<int>::iterator it = roots.begin();
            int ioffset = 0;
            tree_.setRoot(branchVect.at(*it));
            it++;//remain the first one to tree_
            ioffset++;
            vector<int> removeSet(0,0);
            for (;it!=roots.end(); ++it, ioffset++){//for each other tree, pick it's children from tree_
                Tree_ tree_2 (&branchVect, &blockVect, *it, iblock, treeVect.size()+ioffset);
                set<int> index_ = tree_.getIndex();
                int ib ;
                for(set<int>::iterator itt = index_.begin(); itt!= index_.end(); ++itt){
                    ib = *itt;
                    while (roots.find(ib) == roots.end()){//while ib is not one of the root
                        ib = branchVect.at(ib).n;
                    }
                    if (ib == tree_2.getRoot()){
                        //tree_.eraseIndex(*itt);
                        tree_2.insertIndex(branchVect.at(*itt));
                        removeSet.push_back(*itt);
                        if (branchVect.at(*itt).p.IsOnPoly(block)){
                            //tree_.erasePoint(tree_.EP, *itt);
                            tree_2.insertEP(branchVect.at(*itt));
                        }
                    }
                }
                tree_2.buildMaxSegment();
                treeVect.push_back(tree_2);
            }
            while (!removeSet.empty()){//don't remove branch in tree_ when they are needed for nexts
                tree_.eraseIndex(removeSet.back());
                if (branchVect.at(removeSet.back()).p.IsOnPoly(block))
                    tree_.erasePoint(tree_.EP, removeSet.back());
                removeSet.pop_back();
            }

        }
        if (!tree_.empty()){
            //tree_.print();
            tree_.buildMaxSegment();
            treeVect.push_back(tree_);
        }
    }//iteration of blocks
}
void Design::reformBranch (){
    /*--------- Here is make a sequence----*/
    initialSequenceBranch(root);
    /*-------Here is set up EP, FP for tree---*/
    /*--------- split L shape branchs----*/
    tree.buildMaxSegment();
    tree.breakLShape();
    //    print(treeVect); verified
}

void Design::printBlocks() const{
    int iEdge;
    vector<Poly >::const_iterator itt;
    Poly block;
    Point p;
    printf ("Starting printing all poly blocks...\n");
    for (itt = blockVect.begin(); itt != blockVect.end(); itt++){
        block = *itt;
        block.print();
    }
}
//void BlockVect::reform (){
//-1. they are not intersecting rects! they are rects with adjacent edges
//0. not write it now because not so needed
//1. bbox for rect_i, rect_j
//2. identify all points on bbox
//3. if corners of bbox are not included, flip them with previous and behind point
//4. include these flipped points
//5. http://stackoverflow.com/questions/643995/algorithm-to-merge-adjacent-rectangles-into-polygon
//}


void Design::SortAndRecordSlew (){
    float max_slew = 0, min_slew = FLT_MAX;
    for (int i = 0; i < treeVect.size(); ++i){
        treeVect[i].MP = treeVect[i].EP;//MP initially is EP. But at each round in while loop, MP will eliminate the worst branch. EP will not eliminate this branch
        //        treeVect[i].plotTree();
        treeVect[i].PropagateCap();
        treeVect[i].CalculateElmore();//This function will update Elmore at every EP and MP
        PairSecond ps;
        sort (treeVect[i].MP.begin(), treeVect[i].MP.end(), ps);
        max_slew = max (max_slew, treeVect[i].MP[0].second);
        min_slew = min (min_slew, treeVect[i].MP[treeVect[i].MP.size()-1].second);
    }
    slew_spec_ = min_slew + slew_percentage_*(max_slew-min_slew);
    length_limit_ = ( -(RUNIT*Cb+Rs*CUNIT)+sqrt(pow((RUNIT*Cb+Rs*CUNIT),2) - 2*RUNIT*CUNIT*(Rs*Cb-slew_spec_)) ) / (RUNIT*CUNIT);
    cout<< "The min slew is "<< min_slew<< ". The max slew is "<< max_slew<< endl;
}
void Design::legitimate (){
    for (int i = 0; i < treeVect.size(); ++i){
        treeVect[i].MP = treeVect[i].EP;//MP initially is EP. But at each round in while loop, MP will eliminate the worst branch. EP will not eliminate this branch
        //        treeVect[i].plotTree();
        while (!treeVect[i].MP.empty()){
            treeVect[i].PropagateCap();
            treeVect[i].CalculateElmore();//This function will update Elmore at every EP and MP
#ifdef INFO
            treeVect[i].print();
#endif
            PairSecond ps;
            sort (treeVect[i].MP.begin(), treeVect[i].MP.end(), ps);

            float slew0 = treeVect[i].MP[0].second;
            if (slew0 <= slew_spec_) break;//go to next inside tree
            int iworstBranch = treeVect[i].MP[0].first;
            PRS prs(iworstBranch, slew0, slew_spec_, deltaX);
            treeVect[i].buildPR (prs);

            prs.write();
            system ("/home/polaris/yzhang1/Implementation/flute-3.1/flute-3.1/tmp/call_gurobi.csh >> /home/polaris/yzhang1/Implementation/flute-3.1/flute-3.1/tmp/gurobi.log");
            prs.read();
            vector<pair<int,int> > unionR_i; 
            updateTree (i, prs, unionR_i);//after this, unionR contains all abandoned EP on edges
            tree.generateDangling(unionR_i);//after this, unionR contains all need-connection point in tree
            for (int j = 0; j < unionR_i.size(); j++){
                unionR.push_back (unionR_i.back() );
                unionR_i.pop_back();
            }
            //            treeVect[i].plotTree();
        }
    }
    //tree.generateDangling(unionR);//after this, unionR contains all need-connection point in tree
    tree.removeDanglingInUnionR(unionR);
    return;
}

int Design::findBranchOrFP(const Point & p) const{//FP is EP+root
    int ibranch = tree.findBranch(p);
    if (ibranch!=-1) return ibranch;

    int i, j, k;
    Branch branch;
    Point point;
    for (i = 0; i < treeVect.size(); ++i){//for all EP
        for (j = 0; j<treeVect[i].EP.size();++j){
            k = treeVect[i].EP[j].first;
            branch = branchVect.at(k);
            point = branch.p;
            if (p==point)
                return k;
        }
        if ( p == branchVect.at(treeVect[i].getRoot()).p )
            return treeVect[i].getRoot();
    }
    return -1;

}
bool Design::isBranchOrFP(const Point & p) const{//FP is EP+root
    return (findBranchOrFP(p) != -1);
}
bool Design::PointIsInBlocks(const Point & p) const{//strictly in
    vector<Poly >::const_iterator iit;
    int iblock;
    Poly block;
    for (iit = blockVect.begin(); iit != blockVect.end(); ++iit){//for block corners
        iblock = iit-blockVect.begin();
        block = *iit;
        if (block.contain(p))
            return true;
    }
    return false;
}
bool Design::ContainSinkInBlock(const Poly & block) const{//strictly in
    vector<Point>::const_iterator it;
    for (it = sinks_.begin(); it != sinks_.end(); ++it){//for block corners
        Point p = *it;
        if (block.contain(p))
            return true;
    }
    return false;
}
void Design::generateGrid(){
    set<int>::iterator it;
    set<float> tempxgrids, tempygrids;
    vector<Poly >::iterator iit;
    set<int> index;
    Branch branch;
    Point point;
    Edge blEdge;
    int i, j, k, ibranch, iblock;
    Poly block;
    vector<float> vf;
    vector<bool> vb;
    vector<DIRECTION> vi;

    index = tree.getIndex();
    for (it=index.begin(); it!=index.end(); it++) {//for branch in tree
        branch = branchVect.at(*it);
        point = branch.p;
        tempxgrids.insert(point.x);
        tempygrids.insert(point.y);
    }

    for (i = 0; i < treeVect.size(); ++i){//for all EP
        for (j = 0; j<treeVect[i].EP.size();++j){
            k = treeVect[i].EP[j].first;
            branch = branchVect.at(k);
            point = branch.p;
            tempxgrids.insert(point.x);
            tempygrids.insert(point.y);
        }
    }
    vector<pair<int,int> >::iterator it2;
    for (it2=unionR.begin(); it2!=unionR.end(); ++it2){//for Points in unionR
        ibranch = it2->first;
        branch = branchVect.at(ibranch);
        point = branch.p;
        tempxgrids.insert(point.x);
        tempygrids.insert(point.y);
    }

    for (iit = blockVect.begin(); iit != blockVect.end(); ++iit){//for block corners
        iblock = iit-blockVect.begin();
        block = *iit;
        for (i = 0; i < block.size(); ++i){//intersection part
            blEdge = block.edge(i); 
            blEdge.v_? tempxgrids.insert(blEdge.c_):tempygrids.insert(blEdge.c_);
        }
    }

    for (i = 0; i < tempxgrids.size(); i++){
        vf = vector<float> (tempygrids.size(), FLT_MAX);
        vb = vector<bool> (tempygrids.size(), false);
        vi = vector<DIRECTION> (tempygrids.size(), UNREACHED);
        gridMarks.push_back(vf);
        blockMarks.push_back(vb);
        directions.push_back(vi);
    }
    xgrids = std::vector<float> (tempxgrids.begin(), tempxgrids.end());
    ygrids = std::vector<float> (tempygrids.begin(), tempygrids.end());

    for (iit = blockVect.begin(); iit != blockVect.end(); ++iit){//for block corners
        iblock = iit-blockVect.begin();
        block = *iit;
        float xmin = block.at(0).x;
        float ymin = block.at(0).y;
        float xmax = block.at(2).x;
        float ymax = block.at(2).y;
        int ixmin = int (find(xgrids.begin(), xgrids.end(), xmin)-xgrids.begin());
        int ixmax = int (find(xgrids.begin(), xgrids.end(), xmax)-xgrids.begin());
        int iymin = int (find(ygrids.begin(), ygrids.end(), ymin)-ygrids.begin());
        int iymax = int (find(ygrids.begin(), ygrids.end(), ymax)-ygrids.begin());
        for (i = ixmin+1; i<ixmax; i++){//only mark inside,not on edge
            for (j = iymin+1; j < iymax; j++)
                blockMarks[i][j] = true;
        }

    }
#ifdef DEBUG
#endif
}
void Design::traceBack( MRP endMRP){
    /*--------------------handle end newBranch and endBranch-------------------*/
    Point endPoint = endMRP.p;
    int iendBranch = findBranchOrFP (endPoint);//include i, not include n/////////////function
    Branch endBranch = branchVect.at(iendBranch);
    Edge edge = computeEdge (iendBranch);
    if (!endPoint.isEndpoint(edge)){
        Branch newBranch = Branch (branchVect.size(), endPoint, endBranch.n, 0);
        branchVect.push_back(newBranch);
        tree.insertIndex(newBranch);
        branchVect.at(iendBranch).n = newBranch.i;
        endBranch = newBranch;//if this is the case, new created newBranch will be endBranch
    }

    /*--------------------handle middle branch and start branch-------------------*/
    Branch branch;
    int iendXgrid = endMRP.ixgrid, iendYgrid = endMRP.iygrid;
    int ixgrid = iendXgrid, iygrid = iendYgrid;

    Point point = endMRP.p;
    DIRECTION direction = UNREACHED, previousDirection = UNREACHED;
    while (!(direction == UNREACHED && previousDirection!= UNREACHED)){//not (at the end and not the  beginning)
        //ixgrid != istartXgrid || iygrid!= istartYgrid)
        /*-----------updates childPoint, point, ixgrid, iygrid---------------*/
        bool isEnd = ixgrid==iendXgrid && iygrid==iendYgrid;
        previousDirection = direction;
        direction  = directions[ixgrid][iygrid];
        if (direction==RIGHT){
            ixgrid -= 1;
            point.x = xgrids.at(ixgrid);
        }else if (direction==LEFT){
            ixgrid += 1;
            point.x = xgrids.at(ixgrid);
        }else if (direction==UP){
            iygrid -= 1;
            point.y = ygrids.at(iygrid);
        }else if (direction==DOWN){
            iygrid += 1;
            point.y = ygrids.at(iygrid);
        }

        if (isEnd){//first branch from end point
            branch = Branch (branchVect.size(), point, endBranch.i);
        }else{
            if (direction == previousDirection){//keep going
                branch.p = point;
            }else if (direction != UNREACHED){//a turn, but not the end. End has it's own way
                branchVect.push_back(branch);
                tree.insertIndex(branch);
                int itemp = branch.i;
                branch = Branch (branchVect.size(), point, itemp, 0);
            }
        }
        if (direction == UNREACHED){
            /*--------------------handle end newBranch and startBranch-------------------*/
            Point startPoint = point;
            int istartBranch = findBranchOrFP (startPoint);//include i, not include n
            Branch startBranch = branchVect.at(istartBranch);
            Edge edge = computeEdge (istartBranch);
            /*-------------------insert these branchs--------------------------*/
            if (branchVect.at(istartBranch).n == istartBranch){//istartBranch is a point
                branch.i = istartBranch;
                branchVect[istartBranch] = branch;
            }else{//it's either the branch point or in the middle of a branch
                if (!startPoint.isEndpoint(edge)){
                    Branch newBranch = Branch (branchVect.size(), startPoint, startBranch.n, 0);
                    branchVect.at(istartBranch).n = newBranch.i;
                    branchVect.push_back(newBranch);
                    tree.insertIndex(newBranch);

                    istartBranch = newBranch.i;//now startPoint is at branch point
                }
                sequenceBranch(istartBranch);//all branchs and deeper istartBranch point to should point back
                branch.i = istartBranch;
                branchVect[istartBranch] = branch;
            }
            return;
        }
    }//end of while
}
void Design::mazeRouting(){
    generateGrid();
    int i,j, ibranch, itree;
    Branch branch;
    set<int> connectedBranchs;
    set<int>::iterator sit;
    vector<pair<int,int> >::iterator it;
    for (it=unionR.begin(); it!=unionR.end(); ++it){//unionR can be sort then can be faster
        Point startPoint;
        MRP startMRP, endMRP;
        BinaryHeap<float, MRP> mheap;

        ibranch = it->first;
        itree = it->second;
        branch = branchVect.at(ibranch);
        connectedBranchs = tree.findConnectedBranch(ibranch);
        for (sit = connectedBranchs.begin(); sit != connectedBranchs.end(); ++sit){
            Edge edge = computeEdge(*sit);
            if (edge.v_){
                int iygrid_max = int (find(ygrids.begin(), ygrids.end(),
                            edge.c2_)-ygrids.begin());
                int iygrid_min = int (find(ygrids.begin(), ygrids.end(),
                            edge.c1_)-ygrids.begin());
                int ixgrid = int (find(xgrids.begin(), xgrids.end(),
                            edge.c_)-xgrids.begin());
                if (iygrid_min == iygrid_max){//it's a point, (root)
                    startMRP.p = Point(edge.c_, ygrids[iygrid_min]);
                    startMRP.ixgrid = ixgrid;
                    startMRP.iygrid = iygrid_min;
                    startMRP.lengthInBlock = 0;
                    mheap.put(0, startMRP);
                    gridMarks[startMRP.ixgrid][startMRP.iygrid] = 0;
                }else{
                    for (i = iygrid_min; i < iygrid_max; i++){
                        startMRP.p = Point(edge.c_, ygrids[i]);
                        startMRP.ixgrid = ixgrid;
                        startMRP.iygrid = i;
                        startMRP.lengthInBlock = 0;
                        mheap.put(0, startMRP);
                        gridMarks[startMRP.ixgrid][startMRP.iygrid] = 0;
                    }
                }
            }else{
                int ixgrid_max = int (find(xgrids.begin(), xgrids.end(),
                            edge.c2_)-xgrids.begin());
                int ixgrid_min = int (find(xgrids.begin(), xgrids.end(),
                            edge.c1_)-xgrids.begin());
                int iygrid = int (find(ygrids.begin(), ygrids.end(),
                            edge.c_)-ygrids.begin());
                if (ixgrid_min == ixgrid_max){//it's a point, (root)
                    startMRP.p = Point(ixgrid_min, edge.c_);
                    startMRP.ixgrid = ixgrid_min;
                    startMRP.iygrid = iygrid;
                    startMRP.lengthInBlock = 0;
                    mheap.put(0, startMRP);
                    gridMarks[startMRP.ixgrid][startMRP.iygrid] = 0;
                }else{
                    for (i = ixgrid_min; i <= ixgrid_max; i++){
                        startMRP.p = Point(xgrids[i], edge.c_);
                        startMRP.ixgrid = i;
                        startMRP.iygrid = iygrid;
                        startMRP.lengthInBlock = 0;
                        mheap.put(0, startMRP);
                        gridMarks[startMRP.ixgrid][startMRP.iygrid] = 0;
                    }
                }
            }//end of edge veritical or horizontal
        }//end of making a mheap for findpath

        clock_t start, end;
        start = clock();
        pair<float, MRP > pathPair = findPath (mheap, ibranch);//some situation no one call in while loop
        end = clock();
        double cpuTime;
        cpuTime= (end-start)/ (double)(CLOCKS_PER_SEC);
        cout<< "cputTime for this findpath is "<< cpuTime<< endl;
        endMRP = pathPair.second;
        float lengthSoFar = pathPair.first;

        if (!tree.isIndex(ibranch))
            tree.insertIndex(branch);
        traceBack(endMRP);
        clearGridMarksAndDirections();

        //plotAndSaveTree(-1);
    }
}
pair< float, MRP > Design::findPath (BinaryHeap<float, MRP>& mheap, int ibranch){
    int i,j;
    float lengthSoFar, lengthSoFar2;
    MRP mrp, mrp2;

    double cpuTime = 0;
    double cpuTime1 = 0;
    double cpuTime2 = 0;
    double cpuTime3 = 0;
    int counter = 0;
    while (!mheap.isEmpty()){
        counter++;
        clock_t start,start1, start2, start3, end, end1, end2, end3;
        start3 = clock();
        mrp = mheap.min();
        lengthSoFar = mheap.minTag();
        mheap.extractMin();

        if (isBranchOrFP(mrp.p) && !PointIsInBlocks(mrp.p) && !tree.isConnectedBranch(ibranch,findBranch(mrp.p))){//mazerouting may lead to branch in block, cannot connect to these points because may lead to slew failure. Sometimes the startPoint is FP, so need connection to others
            break;
        }
        if (mrp.ixgrid<0 || mrp.ixgrid>=xgrids.size() || //illegal case
                mrp.iygrid<0 || mrp.iygrid>=ygrids.size() ||
                mrp.lengthInBlock > length_limit_)
            continue;
        end3 = clock();
        cpuTime3  += (end3-start3)/ (double)(CLOCKS_PER_SEC);
        /*-------------generate next MazeRoutingPoint-------------*/
        start = clock();
        if (mrp.ixgrid>0){//xgrids-1
            mrp2 = mrp;
            mrp2.ixgrid -= 1; 
            mrp2.p.x = xgrids.at(mrp2.ixgrid);
            lengthSoFar2 = lengthSoFar;
            lengthSoFar2 += mrp.p.x - mrp2.p.x;
            start1 = clock();
            if (blockMarks[mrp.ixgrid][mrp.iygrid] || blockMarks[mrp2.ixgrid][mrp2.iygrid] || blockMarks[(mrp.ixgrid+mrp2.ixgrid)/2][(mrp.iygrid+mrp2.iygrid)/2]){
                mrp2.lengthInBlock +=  mrp.p.x - mrp2.p.x;
                lengthSoFar2 += FACTOR*(mrp.p.x - mrp2.p.x);
            }
            else{
                lengthSoFar2 -= FACTOR*mrp2.lengthInBlock;
                mrp2.lengthInBlock = 0;
            }
            end1 = clock();
            /*-------------put in next MazeRoutingPoint, or update-------------*/
            start2 = clock();
            if (mrp2.lengthInBlock < length_limit_){
                if (gridMarks[mrp2.ixgrid][mrp2.iygrid] > lengthSoFar2 && !tree.isConnectedBranch(ibranch, findBranchOrFP(mrp2.p))){
                    if (gridMarks[mrp2.ixgrid][mrp2.iygrid] == FLT_MAX)
                        mheap.put(lengthSoFar2, mrp2);
                    else
                        mheap.decrease(lengthSoFar2, mrp2);
                    gridMarks[mrp2.ixgrid][mrp2.iygrid] = lengthSoFar2;
                    directions[mrp2.ixgrid][mrp2.iygrid] = LEFT;
                }
                /*else {
                    bool changed = (mheap.decrease(lengthSoFar, mrp2));////////////function, find by ixgrid, iygrid, or p, need test
                    if (changed)
                        directions[mrp2.ixgrid][mrp2.iygrid] = LEFT;
                }*/
            }
            end2 = clock();
        }
        end = clock();
        cpuTime += (end-start)/ (double)(CLOCKS_PER_SEC);
        cpuTime1 += (end1-start1)/ (double)(CLOCKS_PER_SEC);
        cpuTime2 += (end2-start2)/ (double)(CLOCKS_PER_SEC);
        start = clock();
        if (mrp.ixgrid<xgrids.size()-1){//ixgrid+1
            mrp2 = mrp;
            mrp2.ixgrid += 1; 
            mrp2.p.x = xgrids.at(mrp2.ixgrid);
            lengthSoFar2 = lengthSoFar;
            lengthSoFar2 += mrp2.p.x - mrp.p.x;
            start1 = clock();
            if (blockMarks[mrp.ixgrid][mrp.iygrid] || blockMarks[mrp2.ixgrid][mrp2.iygrid] || blockMarks[(mrp.ixgrid+mrp2.ixgrid)/2][(mrp.iygrid+mrp2.iygrid)/2]){
                mrp2.lengthInBlock +=  mrp2.p.x - mrp.p.x;
                lengthSoFar2 += FACTOR*(mrp.p.x - mrp2.p.x);
            }
            else{
                lengthSoFar2 -= FACTOR*mrp2.lengthInBlock;
                mrp2.lengthInBlock = 0;
            }
            end1 = clock();
            /*-------------put in next MazeRoutingPoint, or update-------------*/
            start2 = clock();
            if (mrp2.lengthInBlock < length_limit_){
                if (gridMarks[mrp2.ixgrid][mrp2.iygrid] > lengthSoFar2 && !tree.isConnectedBranch(ibranch, findBranchOrFP(mrp2.p))){
                    if (gridMarks[mrp2.ixgrid][mrp2.iygrid] == FLT_MAX)
                        mheap.put(lengthSoFar2, mrp2);
                    else
                        mheap.decrease(lengthSoFar2, mrp2);
                    gridMarks[mrp2.ixgrid][mrp2.iygrid] = lengthSoFar2;
                    directions[mrp2.ixgrid][mrp2.iygrid] = RIGHT;
                }
                /*else {
                    bool changed = (mheap.decrease(lengthSoFar, mrp2));////////////function, find by ixgrid, iygrid, or p, need test
                    if (changed)
                        directions[mrp2.ixgrid][mrp2.iygrid] = RIGHT;
                }*/
            }
            end2 = clock();
        }
        end = clock();
        cpuTime += (end-start)/ (double)(CLOCKS_PER_SEC);
        cpuTime1 += (end1-start1)/ (double)(CLOCKS_PER_SEC);
        cpuTime2 += (end2-start2)/ (double)(CLOCKS_PER_SEC);
        start = clock();
        if (mrp.iygrid>0){//ygrids-1
            mrp2 = mrp;
            mrp2.iygrid -= 1; 
            mrp2.p.y = ygrids.at(mrp2.iygrid);
            lengthSoFar2 = lengthSoFar;
            lengthSoFar2 += mrp.p.y - mrp2.p.y;
            start1 = clock();
            if (blockMarks[mrp.ixgrid][mrp.iygrid] || blockMarks[mrp2.ixgrid][mrp2.iygrid] || blockMarks[(mrp.ixgrid+mrp2.ixgrid)/2][(mrp.iygrid+mrp2.iygrid)/2]){
                mrp2.lengthInBlock +=  mrp.p.y - mrp2.p.y;
                lengthSoFar2 += FACTOR*(mrp.p.x - mrp2.p.x);
            }
            else{
                lengthSoFar2 -= FACTOR*mrp2.lengthInBlock;
                mrp2.lengthInBlock = 0;
            }
            end1 = clock();
            /*-------------put in next MazeRoutingPoint, or update-------------*/
            start2 = clock();
            if (mrp2.lengthInBlock < length_limit_){
                if (gridMarks[mrp2.ixgrid][mrp2.iygrid] > lengthSoFar2 && !tree.isConnectedBranch(ibranch, findBranchOrFP(mrp2.p))){
                    if (gridMarks[mrp2.ixgrid][mrp2.iygrid] == FLT_MAX)
                        mheap.put(lengthSoFar2, mrp2);
                    else
                        mheap.decrease(lengthSoFar2, mrp2);
                    gridMarks[mrp2.ixgrid][mrp2.iygrid] = lengthSoFar2;
                    directions[mrp2.ixgrid][mrp2.iygrid] = DOWN;
                }
               /* else {
                    bool changed = (mheap.decrease(lengthSoFar, mrp2));////////////function, find by ixgrid, iygrid, or p, need test
                    if (changed)
                        directions[mrp2.ixgrid][mrp2.iygrid] = DOWN;
                }*/
            }
            end2 = clock();
        }
        end = clock();
        cpuTime += (end-start)/ (double)(CLOCKS_PER_SEC);
        cpuTime1 += (end1-start1)/ (double)(CLOCKS_PER_SEC);
        cpuTime2 += (end2-start2)/ (double)(CLOCKS_PER_SEC);
        start = clock();
        if (mrp.iygrid<ygrids.size()-1){//iygrid+1
            mrp2 = mrp;
            mrp2.iygrid += 1; 
            mrp2.p.y = ygrids.at(mrp2.iygrid);
            lengthSoFar2 = lengthSoFar;
            lengthSoFar2 += mrp2.p.y - mrp.p.y;
            start1 = clock();
            if (blockMarks[mrp.ixgrid][mrp.iygrid] || blockMarks[mrp2.ixgrid][mrp2.iygrid] || blockMarks[(mrp.ixgrid+mrp2.ixgrid)/2][(mrp.iygrid+mrp2.iygrid)/2]){
                mrp2.lengthInBlock +=  mrp2.p.y - mrp.p.y;
                lengthSoFar2 += FACTOR*(mrp.p.x - mrp2.p.x);
            }
            else{
                lengthSoFar2 -= FACTOR*mrp2.lengthInBlock;
                mrp2.lengthInBlock = 0;
            }
            end1 = clock();
            /*-------------put in next MazeRoutingPoint, or update-------------*/
            start2 = clock();
            if (mrp2.lengthInBlock < length_limit_){
                if (gridMarks[mrp2.ixgrid][mrp2.iygrid] > lengthSoFar2 && !tree.isConnectedBranch(ibranch, findBranchOrFP(mrp2.p))){
                    if (gridMarks[mrp2.ixgrid][mrp2.iygrid] == FLT_MAX)
                        mheap.put(lengthSoFar2, mrp2);
                    else
                        mheap.decrease(lengthSoFar2, mrp2);
                    gridMarks[mrp2.ixgrid][mrp2.iygrid] = lengthSoFar2;
                    directions[mrp2.ixgrid][mrp2.iygrid] = UP;
                }
                /*else {
                    bool changed = (mheap.decrease(lengthSoFar, mrp2));////////////function, find by ixgrid, iygrid, or p, need test
                    if (changed)
                        directions[mrp2.ixgrid][mrp2.iygrid] = UP;
                }*/
            }
            end2 = clock();
        }
        end = clock();
        cpuTime += (end-start)/ (double)(CLOCKS_PER_SEC);
        cpuTime1 += (end1-start1)/ (double)(CLOCKS_PER_SEC);
        cpuTime2 += (end2-start2)/ (double)(CLOCKS_PER_SEC);

    }// end of while
    cout<< "In findpath, "<< counter<< "nodes out from heap"<< endl;
    cout<< "cputTime in findpath is "<< cpuTime<< endl;
    cout<< "cputTime1 in findpath is "<< cpuTime1<< endl;
    cout<< "cputTime2 in findpath is "<< cpuTime2<< endl;
    cout<< "cputTime3 in findpath is "<< cpuTime3<< endl;

//    assert (mheap.getSize() > 0);
    if (mheap.getSize() == 0){//this startMRP is unable to reach any tree without step on its connection and too much blockage
        return pair<float, MRP> (FLT_MAX, mrp);
    }
    return pair<float, MRP > (lengthSoFar, mrp);
}
void Design::clean(){
    unionR.clear();
}
void Design::uniteAllTrees(){
    int i;
    for (i = 0; i < treeVect.size(); ++i){
        tree.unite(treeVect[i]);
    }
//    tree.print();
}
int Design::findBranch(const Point& point)const {
    Edge edge;
    int ibranch;
    int i;
    ibranch = tree.findBranch(point);
    if (ibranch != -1)   return ibranch;
    for (i = 0; i < treeVect.size(); ++i){
       ibranch = treeVect[i].findBranch(point); 
       if (ibranch != -1)   return ibranch; 
    }
    for (i = 0; i < unionR.size(); ++i){
        ibranch = unionR[i].first;
        edge = computeEdge (ibranch);
        if (point.IsOnEdge(edge)){//unionR contains single point
            return ibranch;
        }
    }
    return -1;
}
/*--------------  CLASS functions   ------------------*/
DTYPE Point::distance (const Edge& e) const{//return the shortest Manhatan distance to e
    if (e.v_){
        if (y<e.c1_) return abs(x-e.c_)+abs(e.c1_-y);
        else if (y>e.c2_) return abs(x-e.c_)+abs(y-e.c2_);
        else return abs(x-e.c_);
    }else{
        if (x<e.c1_) return abs(y-e.c_)+abs(e.c1_-x);
        else if (x>e.c2_) return abs(y-e.c_)+abs(x-e.c2_);
        else return abs(y-e.c_);
    }
}
Point Point::projection (const Edge& e) const{
    if (e.v_){
        assert (y<=e.c2_ && y>=e.c1_);
        return (Point(e.c_,y));
    }else {
        assert (x<=e.c2_ && x>=e.c1_);
        return (Point(x,e.c_));
    }
}

bool Point::isEndpoint (const Edge& edge)const{
    return *this==edge.point1()||*this==edge.point2();
}

void Poly::Plot()const{//use gnu to plot a polyhedron
    FILE * fp = popen ("gnuplot -persist", "w");
    fprintf (fp, "set object 1 polygon from ");
    float x_min = FLT_MAX, x_max = 0, y_min = FLT_MAX, y_max = 0;
    for (int i=0; i<size();++i){
        fprintf (fp, "%f,%f to ", (float)at(i).x, (float)at(i).y);
        x_min = min(at(i).x, x_min);
        x_max = max(at(i).x, x_max);
        y_min = min(at(i).y, y_min);
        y_max = max(at(i).y, y_max);
    }
    fprintf (fp, " %f,%f fs pattern 1 bo 2 fc rgbcolor \"cyan\"\n",  (float)at(0).x, (float)at(0).y);//to close the Polygon
    fprintf (fp, "set xrange [ %f : %f ]\n", x_min-0.2*(x_max-x_min), x_max+0.2*(x_max-x_min));
    fprintf (fp, "set yrange [ %f : %f ]\n", y_min-0.2*(y_max-y_min), y_max+0.2*(y_max-y_min));
    fprintf (fp, "plot x with dots\n");//plot something
    fprintf (fp, "set terminal x11\n");
    fprintf (fp, "pause -1\n");
    pclose (fp);

}
void Design::plotAndSaveTree(int itree)const{//generate line doc for gnu
    Tree plotedTree = (itree==-1)?tree:treeVect[itree];
    set<int> index = plotedTree.getIndex();
    static int count = -1;
    count++;//every time plotTree got called, count+1
    int i;
    Poly block;
    vector<Poly >::const_iterator iit;
    FILE * fp;
    string str= "./tmp/tree.branch";
    if( (fp=fopen(str.c_str(),"w")) == NULL ) {
        sprintf(error_text, "bookshelf_IO: Cannot open: %s file for write", str.c_str());
        runtimeError(error_text);
    }
    Branch branch;
    set<int>::iterator it, it2;
    for (it=index.begin(); it!=index.end(); it++) {
        branch = branchVect.at(*it);
        fprintf(fp, "%f %f ", (float)branch.p.x,(float)branch.p.y);
        fprintf(fp, "%f %f\n", (float)branchVect.at(branch.n).p.x, (float)branchVect.at(branch.n).p.y);
    }
    fclose(fp);

    fp = popen ("gnuplot -persist", "w");
//    fp=fopen(str.c_str(),"w");
    for (it=index.begin(); it!=index.end(); it++) {
        branch = branchVect.at(*it);
        fprintf(fp, "set arrow from %f,%f to %f,%fnohead\n", (float)branch.p.x,(float)branch.p.y, (float)branchVect.at(branch.n).p.x, (float)branchVect.at(branch.n).p.y);
    }
    for (iit = blockVect.begin(); iit != blockVect.end(); ++iit){
        block = *iit;
        fprintf (fp, "set object %d polygon from ", 1 + iit-blockVect.begin());//from 1
        for (i=0; i<block.size();++i){
            fprintf (fp, "%f,%f", (float)block.at(i).x, (float)block.at(i).y);
            fprintf (fp," to ");
        }
        fprintf (fp, " %f,%f fs pattern 1 bo 2 fc rgbcolor \"cyan\"\n",  (float)block.at(0).x, (float)block.at(0).y);//to close the Polygon
    }
    fprintf (fp, "set xrange [ %f : %f ]\n", xmin_-0.2*(xmax_-xmin_), xmax_+0.2*(xmax_-xmin_));
    fprintf (fp, "set yrange [ %f : %f ]\n", ymin_-0.2*(ymax_-ymin_), ymax_+0.2*(ymax_-ymin_));
    if (index.size() != 0)//if index size is 0, then no branch printed
        fprintf (fp, "plot ");
    for (it=index.begin(); it!=index.end(); ) {
        branch = branchVect.at(*it);
        if (branch.i == root)
            fprintf(fp, "\"<echo '%f %f'\" notitle with points pointtype 7 pointsize 2 linecolor rgb \"red\"", (float)branch.p.x, (float)branch.p.y);
        else
            fprintf(fp, "\"<echo '%f %f'\" notitle with points pointtype 4 pointsize 2 linecolor rgb \"blue\"",(float)branch.p.x, (float)branch.p.y);
        it++;
        if (it != index.end())    fprintf (fp, ", ");
        else    fprintf (fp, "\n");
    }
    fprintf (fp, "set terminal png\n");  
    //fprintf (fp, "set output \"./tmp/my-plot.png\"\n");//gthumb print
    fprintf (fp, "set output \"%s/my_plot_%d.png\"\n", benchmarkPath, count);//gthumb Erint
    if (index.size() != 0)
        fprintf (fp, "replot\n");
    else
        fprintf (fp, "plot x with dots\n");//plot something
    fprintf (fp, "set terminal x11\n");
    fprintf (fp, "pause -1\n");
    pclose (fp);

}


pair<float, float> Design::getOutsideWL() const{
    set<int> index = tree.getIndex();
    Edge edge;
    Branch branch;
    float outsideWL = 0;
    float insideWL = 0;
    set<int>::iterator it;
    for (it=index.begin(); it!=index.end(); it++) {
        branch = branchVect.at(*it);
        edge = Edge (branch.p, branchVect.at(branch.n).p);
        if (PointIsInBlocks(edge.point1()) || PointIsInBlocks(edge.point2()))//tree may have inside branch because of mazerouting
            insideWL += edge.length();
        else{
            outsideWL += edge.length();
//            printf ("from br%d to br%d, increase outsideWL %f, now total is %f\n", branch.i, branch.n, edge.length(), outsideWL);
        }
    }
    return pair<float,float> (insideWL,outsideWL);
}
float Design::getInsideWL() const{
    int i;
    const Tree* ptree;
    float insideWL = 0;
    for (i = 0; i < treeVect.size(); ++i){
        ptree = &treeVect[i];
        set<int> index = ptree->getIndex();
        Edge edge;
        Branch branch;
        set<int>::iterator it, it2;
        for (it=index.begin(); it!=index.end(); it++) {
            branch = branchVect.at(*it);
            edge = Edge (branch.p, branchVect.at(branch.n).p);
            insideWL += edge.length();
        }
    }
    return insideWL;
}

bool Point::isProjectedIn (const Edge& e) const{
    if (e.v_){
        return (y<e.c2_ && y>e.c1_);
    }else {
        return (x<e.c2_ && x>e.c1_);
    }
}
Point Point::findClosestPoint (const Edge& e) const{
    if (isProjectedIn(e))
        return projection(e);
    else
        return distance(e.point1())<distance(e.point2())? 
            e.point1(): e.point2();
}
bool Point::IsOnEdge (const Edge& edge) const{
    if (edge.v_ == true)
        return (abs(x-edge.c_)<EPSILON) && (edge.c1_<=y) && (y<=edge.c2_);
    else return (abs(y-edge.c_)<EPSILON) && (edge.c1_<=x) && (x<=edge.c2_);
}
bool Point::IsOnPoly (const Poly& poly) const{
    int i;
    for (i = 0; i < poly.size(); ++i){
        Edge edge = poly.edge(i);
        if (IsOnEdge(edge))
            return true;
    }
    return false;
}
bool Point::isOutside (const Poly& poly) const{
    return !(IsOnPoly(poly) || poly.contain(*this));
}
/*---------------------------  Tree functions   ------------------------------*/

void Tree::print() const{
    set<int>::iterator it;
    int i;
    printf ("Starting printing all branchs in Tree...\n");
    printf ("The root of the tree is %d, with downstream cap %f\n", root, pbranchVect->at(root).Ct);
    for (it = index.begin(); it != index.end(); ++it) {
        i = *it;
        printf ("Branch %d from Point (%f,%f) to Branch %d with downstream cap %f is inside Tree\n", i, (float)pbranchVect->at(i).p.x,  (float)pbranchVect->at(i).p.y, pbranchVect->at(i).n, pbranchVect->at(i).Ct );
    }
}
void Tree_::print() const{
#ifdef DEBUG
    printf ("Starting printing the Tree within block %d...\n", iblock);
    Tree::print();
#endif
    int i;
    for (i = 0; i < EP.size(); ++i) {
        printf ("The EP%d is at Branch %d, with slew %f\n", i, EP[i].first, EP[i].second );
    }
    printf ("\n");
}

void Tree::unite (const Tree & that){
    set<int> index2 = that.getIndex();
    set<int>::iterator it;
    for (it = index2.begin(); it != index2.end(); ++it){
        if (!isIndex(*it))
            index.insert (*it);
    }
}
pair<bool,DTYPE> Tree::buildMaxSegment(){
    set<int>::iterator it;
    int ibranch;
    Branch branch;
    Point p, pstart, pend;
    Edge e;
    map <pair< pair<bool,DTYPE>,DIRECTION>, int> m; // <vertical, x, RIGHT/LEFT/UP/DOWN>,repeated times
    map < pair<bool,DTYPE>, int> m2; // when same number, use same  position for choice
    make_tuple (1,2,3);
    map <pair< pair<bool,DTYPE>,DIRECTION>, int>::iterator mit; // <vertical, x>,repeated times
    map <pair<bool,DTYPE>, int>::iterator mit2; // <vertical, x>,repeated times
    DTYPE maxCount = 0, start=INT_MAX, end = INT_MIN;
    pair<bool,DTYPE> maxSeg;

    /*-------------1 EP 1 root situation, use the branch adjacent to root as MaxSeg---------------*/
    if (EP.size() == 1){
        for (it = index.begin(); it != index.end(); ++it){
            ibranch = *it;
            branch = pbranchVect->at(ibranch);
            if (branch.n == root){
                e =  computeEdge(ibranch);//p of branch and its neighbor
                ms = e;
                return pair<bool,DTYPE> (e.v_, e.c_);    
            }
        }

    }
    /*-------------more than 1 EP-----------------------------------------*/
    for (it = index.begin(); it != index.end(); ++it){
        ibranch = *it;
        branch = pbranchVect->at(ibranch);
        p = pbranchVect->at(branch.n).p;
        if ( (branch.p).x != p.x && (branch.p).y != p.y ) //L shape
            continue;
        if (branch.n == root && branch.i == root) //root, then branch.p == p
            continue;
        e =  computeEdge(ibranch);//p of branch and its neighbor
        pair<pair<bool,DTYPE>,DIRECTION> key(make_pair(e.v_, e.c_),e.direction());
        pair<bool,DTYPE> key2(make_pair(e.v_, e.c_));
        mit = m.find(key);
        mit2 = m2.find(key2);
        if (mit == m.end())
            m.insert (pair< pair<pair<bool,DTYPE>, DIRECTION>,int> (key,1));
        else{
            (mit->second)++;
        }
        if (mit2 == m2.end())
            m2.insert ( pair<pair<bool,DTYPE>, int> (key2,1));
        else{
            (mit2->second)++;
        }
    }
    for (mit=m.begin(); mit!=m.end(); ++mit){//looking for most number of same direction and position. If number is same, then depend on 
        if (mit->second > maxCount){
            maxCount = mit->second;
            maxSeg = (mit->first).first;
        }else if (mit->second == maxCount && *(m2.find((mit->first).first)) > *(m2.find(maxSeg))  ){
            maxCount = mit->second;
            maxSeg = (mit->first).first;
        }

    }
    for (it = index.begin(); it != index.end(); ++it){//set start and end of ms
        ibranch = *it;
        branch = pbranchVect->at(ibranch);
        p = pbranchVect->at(branch.n).p;
        if (! branch.p.PointsOnSameLine(p) ) continue;//L shape, only see when this function called by ptree
        e=Edge (branch.p, p);//p of branch and its neighbor
        if (abs(e.v_- maxSeg.first)<EPSILON && abs(e.c_-maxSeg.second)<EPSILON){
            if (e.c1_ < start) start = e.c1_;
            if (e.c2_ > end) end = e.c2_;
        }
    }
    ms = Edge (maxSeg.first, maxSeg.second, start, end);
    return maxSeg;
}
void Tree::breakLShape(){//the break rule is always to extend maxSeg.
    vector<Branch>::size_type ib;
    Branch branch, newBranch;
    Point p;
    Point corner;
    for (ib = 0; ib < pbranchVect->size(); ++ib){
        branch = pbranchVect->at(ib);
        p = pbranchVect->at(branch.n).p;
        if ( (branch.p).x != p.x && (branch.p).y != p.y ){ //L shape
            if (ms.v_){//this is not too much sense in it
                corner.x = p.x;
                corner.y = branch.p.y;
            }
            else{
                corner.y = p.y;
                corner.x = branch.p.x;
            }
            newBranch = Branch (pbranchVect->size(), corner, branch.n);
            pbranchVect->at(ib).n = newBranch.i;
            pbranchVect->push_back (newBranch);
            insertIndex(newBranch);
        }
    }
}

DTYPE Tree::getLength(int ibranch, int ibranch2){//the total length from p of branch to ancestor p of ibranch2
    Point p, p2;
    Branch branch;
    int icurrentBranch = ibranch;
    DTYPE length = 0;
    while (icurrentBranch != ibranch2){
        branch = pbranchVect->at(icurrentBranch);
        p = branch.p;
        p2 = pbranchVect->at(branch.n).p;
        length += p.distance(p2);
        icurrentBranch = branch.n;
    }
    return length;
}

void Tree::generateDangling(vector<pair<int,int> >& unionR){//only removed last branch
    vector<pair<int,int> > unionR2;
    Branch branch;
    pair<int,int> dangling;
    int ibranch, iupBranch;
    set<int>::iterator it;
    vector<int> removeSet(0,0);
    while (!unionR.empty()){
        dangling = unionR.back();
        unionR.pop_back();
        if (CountChildren(dangling.first) == 0){//EP itself is leaf node. Or a root of another tree
//            This is impossible now because in updateTree we add a newBranch for this situation
            pbranchVect->at(dangling.first).n = dangling.first;
            unionR2.push_back(dangling);
        }
        else{
            for (it=index.begin(); it!=index.end(); it++){
                iupBranch = *it; 
                if (pbranchVect->at(iupBranch).n == dangling.first){
                    pbranchVect->at(iupBranch).n = iupBranch;//next to itself, then no more illegal edge
                    unionR2.push_back(pair<int,int> (iupBranch,dangling.second) );//put upBranch number and branch step into a pair
                    removeSet.push_back(iupBranch);
                }
            }
        }
        while (!removeSet.empty()){
            eraseIndex(removeSet.back());
            removeSet.pop_back();
        }

    }
    unionR = unionR2;//now unionR is only with leaf nodes
}
void Tree::removeDanglingInUnionR(vector<pair<int,int> >& unionR){//remove dangling in unionR which has only one child
    vector<pair<int,int> >::size_type i;
    int ibranch, ichildBranch;
    for (i = 0; i< unionR.size();++i){
        ibranch = unionR.at(i).first;
        if (CountChildren(ibranch) == 1 && !isFP(ibranch)){//if it is leaf or root, remove it will lost it finally
            ichildBranch = findOnlyChild(ibranch);
            pbranchVect->at(ichildBranch).n = pbranchVect->at(ichildBranch).i;
            unionR.at(i) = pair<int,int> (pbranchVect->at(ichildBranch).i, unionR.at(i).second) ;
            eraseIndex(ichildBranch);
            i--;
        }
    }
}
int Tree::findBranch (const Point& p)const{
    if (pbranchVect->at(root).p == p)   return root;//this is contradictary with p != pbranchVect->at(branch.n).p
    set<int>::iterator it;
    int ibranch;
    Branch branch;
    Edge edge;
    for (it=index.begin(); it!=index.end();++it){
        branch = pbranchVect->at(*it);
        ibranch = branch.i;
        if (p == branch.p)  
            return ibranch;
        edge = computeEdge (ibranch);
        if (p.IsOnEdge(edge) && p != pbranchVect->at(branch.n).p){//no matter p is at frontend or backend, these two will lead to a 0 length newBranch
            return ibranch;
        }
    }
    return -1;//this should never happen
}
void Tree::trim(){
    removeUseless();
    combine();
    removeZeroLength();
}
int Tree::CountChildren (int ibranch)const{
    set<int>::iterator it;
    int ibranch2;
    int counter = 0;
    for (it=index.begin(); it!=index.end();++it){
        ibranch2 = *it;
        if (pbranchVect->at(ibranch2).n == ibranch && ibranch2!= ibranch)//the root of the steiner tree point to itself
            counter++;
    }
    return counter;

} 
int Tree::findOnlyChild (int ibranch)const{
    assert (CountChildren(ibranch)==1);
    set<int>::iterator it;
    int ibranch2;
    for (it=index.begin(); it!=index.end();++it){
        ibranch2 = *it;
        if (pbranchVect->at(ibranch2).n == ibranch && ibranch2!= ibranch)
            return ibranch2;
    }
} 
void Tree::removeUseless(){
    set<int>::iterator it, it2;
    int ibranch, ibranch2;
    Branch branch, branch2;
    Edge edge;
    bool isDone = false, isUseless;
    vector<int> removeSet(0,0);
    while (!isDone){
        isDone = true;
        for (it=index.begin(); it!=index.end();++it){
            ibranch = *it;
            isUseless = false;
            if (!isFP(ibranch)){// is not fixed points
                isUseless = true;
                if (CountChildren(ibranch) > 0)
                    isUseless= false;
            }
            if (isUseless){
                removeSet.push_back(ibranch);
                isDone = false;
            }
        }
        while (!removeSet.empty()){
            eraseIndex(removeSet.back());
            removeSet.pop_back();
        }
    }

}
void Tree::combine(){
    set<int>::iterator it;
    int ibranch, iparentBranch;
    Branch branch, parentBranch;
    Edge edge, parentEdge;
    bool isDone = false;
    vector<int> removeSet(0,0);
    while (!isDone){
        isDone = true;
        for (it=index.begin(); it!=index.end();++it){
            ibranch = *it;
            branch = pbranchVect->at(ibranch);
            edge = computeEdge(ibranch);
            iparentBranch = branch.n;
            parentBranch = pbranchVect->at(iparentBranch);
            parentEdge = computeEdge(iparentBranch);
            if (isIndex(iparentBranch) && !isFP(iparentBranch) && 
                    CountChildren(iparentBranch) == 1 &&  edge.v_==parentEdge.v_){//combine same line
                pbranchVect->at(ibranch).n = parentBranch.n;
                removeSet.push_back(iparentBranch);
                isDone = false;
            }
        }
        while (!removeSet.empty()){
            eraseIndex(removeSet.back());
            removeSet.pop_back();
        }

    }
    return;
}
void Tree::removeZeroLength(){
    set<int>::iterator it, it2;
    int ibranch, ichildBranch;
    Branch branch, childBranch;
    bool isDone = false;
    vector<int> removeSet(0,0);
    while (!isDone){
        isDone = true;
        for (it=index.begin(); it!=index.end();++it){
            ibranch = *it;
            branch = pbranchVect->at(ibranch);
            if (computeEdge(ibranch).length() == 0){//exactly==0
                for (it2=index.begin(); it2!=index.end();++it2){
                    ichildBranch = *it2;
                    childBranch = pbranchVect->at(ichildBranch);
                    if (childBranch.n==ibranch){
                        pbranchVect->at(ichildBranch).n = branch.n; 
                    }
                }
                removeSet.push_back(ibranch);
                isDone = false;
            }
        }
        while (!removeSet.empty()){
            eraseIndex(removeSet.back());
            removeSet.pop_back();
        }
    }
    return;

}
void Tree::CleanFloatingTerminal(int iparentBranch){//the branch point to iparentBranch should already be removed
    int useTimes;
    set<int>::iterator it;
    int isiblingBranch;//potential sibling index
    while (iparentBranch != getRoot()){//remove unused branch
        useTimes = 0;
        for (it=index.begin(); it!=index.end();it++){
            if (pbranchVect->at(*it).n==iparentBranch) {
                isiblingBranch = *it;
                useTimes += 1;
                continue;//has another child
            }
        }
        if (useTimes==0){//remove dangling branch
            eraseIndex(iparentBranch);
            iparentBranch = pbranchVect->at(iparentBranch).n;
        }else{
            break;
        }
    }
}

bool Tree::isConnectedBranch(int index_branch, int index_branch2)const{//same root
    if (index_branch == -1 || index_branch2 == -1)//?not aware of this, shouldn't be here
        return false;
    while (index_branch != pbranchVect->at(index_branch).n)
        index_branch = pbranchVect->at(index_branch).n;
    while (index_branch2 != pbranchVect->at(index_branch2).n)
        index_branch2 = pbranchVect->at(index_branch2).n;
    return index_branch == index_branch2;
}
set<int> Tree::findConnectedBranch(int ibranch)const{
    set<int> connectedBranchs;
    connectedBranchs.insert(ibranch);
    set<int>::iterator it;
    for (it=index.begin(); it!=index.end(); ++it){
        if (isConnectedBranch(*it, ibranch))
            connectedBranchs.insert(*it);
    }
    return connectedBranchs;
}

/*---------------------------  Tree_ functions   -----------------------------*/
void Tree_::addNonlinearPR (float fbase, float& fbase2, DTYPE dWLbase, DTYPE& dWLbase2, float& fCa, float& fRb, const float&fRc, const float& fCc, Point pstart, Point pend, Branch branch, PR & pr, bool bendOfMaxSeg, DTYPE deltaX){
    //fCa, fRb
    Edge edge(pstart, pend);
    int count = ceil(edge.length()/deltaX);
    int i;
    Point pdelta = (pend  - pstart) * ((float)deltaX/edge.length());
    Point pleft, pright = pstart;
    DTYPE deltaL;//the real step
    float fslope, fCap;

    for (i=0; i<count; i++){
        pleft = pright;
        pright = (i==count-1)?pend: pleft+pdelta;
        deltaL = (i==count-1)?edge.length()-(count-1)*deltaX : deltaX;
        fCap = bendOfMaxSeg? fCc*RUNIT+fRb*CUNIT: (fCa+fCc)*RUNIT;
        fslope = bendOfMaxSeg? -(fCap-0.5*deltaL*CUNIT)*RUNIT: -fCap*RUNIT; //omit 0.5*deltaL*CUNIT in not bendOfMaxSeg, because this will make X bigger, then slew will under slew_spec_

        pr.add (fbase, fslope, deltaL, dWLbase, (fRb-Rs)/RUNIT, bendOfMaxSeg, pleft, pright);
#ifdef DEBUG
        printf ("AddNonLinear: branch%d , pstart(%f,%f), base is %f, WLbase is %f\n", branch.i, (float)pleft.x, (float)pleft.y, fbase, (float)dWLbase);
#endif
        if (bendOfMaxSeg){
            fbase += deltaL*fslope;
        }
        else{
            dWLbase += deltaL;
            fbase += - deltaL*(fCap+0.5*deltaL*CUNIT)*RUNIT;
            fCa += deltaL * CUNIT;
        }
        fRb -= deltaL*RUNIT;
    }

    fbase2 = fbase;
    dWLbase2 = dWLbase;//if bendOfMaxSeg, dWLbase2 = dWLbase. Otherwise dWLbase2 = dWLbase+edge.length()
}

void Tree_::addCombinationPR (const PRS &prs, PR & pr, int ibranch, int inextBranch, int iworstBranch, int ibranchOnMaxSeg, bool bendOfMaxSeg, int startEdge, int stopEdge){
    DTYPE dWLbase = getOutLength(pr.step, ibranch, inextBranch, startEdge, stopEdge); 
    if (bendOfMaxSeg){
        int ineighborBranchOnMaxSeg = pbranchVect->at(pbranchVect->at(ibranchOnMaxSeg).n).i;
        dWLbase -= getLength(ibranchOnMaxSeg, ineighborBranchOnMaxSeg);
    }
    float fbase;
    if (ibranch == iworstBranch){
        fbase = -prs.slew0;
    }else{
        int first_steiner_branch = FindFirstSteinerBranch(ibranch);
        int first_common_ancestor = FindFirstCommonAncestor(first_steiner_branch, iworstBranch);
        DTYPE length_common_R_path = getLength (first_common_ancestor, root);
        DTYPE length_removed_C_path = getLength (ibranch, first_steiner_branch);
        Branch branch = pbranchVect->at(ibranch);
        fbase = -(length_removed_C_path*CUNIT + branch.Ct)*(RUNIT*length_common_R_path + Rs);
    }
    pr.add (fbase,0,0,dWLbase, 0, false, pbranchVect->at(inextBranch).p, pbranchVect->at(inextBranch).p);
#ifdef DEBUG
    printf ("Target branch is %d, branch%d combined with next branch%d, base is %f, WLbase is %f\n",iworstBranch, ibranch, inextBranch, fbase, (float)dWLbase);
#endif
}
DTYPE Tree_::getOutLength(int step, int ibranch, int inextBranch, int startEdge, int stopEdge)const {//the total length on blockage
    int iEdge;
    Edge blEdge;
    Point p = pbranchVect->at(ibranch).p;
    Poly block = pblockVect->at(iblock);
    DTYPE outWL = 0;
    for (iEdge = startEdge; iEdge != stopEdge; iEdge=(iEdge+step+block.size())%block.size()){
        blEdge = block.edge(iEdge); 
        if (iEdge==startEdge){
            if (step==1)
                blEdge = Edge(pbranchVect->at(ibranch).p, block.p2_edge(iEdge));
            else
                blEdge = Edge(pbranchVect->at(ibranch).p, block.p1_edge(iEdge));
        }
        outWL += blEdge.length();
    }
    if (step==1)
        blEdge = Edge(pbranchVect->at(inextBranch).p, block.p1_edge(stopEdge));
    else
        blEdge = Edge(pbranchVect->at(inextBranch).p, block.p2_edge(stopEdge));
    outWL += blEdge.length();
    return outWL;
}
void Tree_::PropagateCap(){
    set<int>::iterator it;
    int ibranch;
    float cap = 0;
    Branch branch;
    DTYPE distance;
    Edge e;
    int root = getRoot();
    for (it=index.begin(); it!=index.end();++it){//initialize to 0
        ibranch = *it;
        branch = pbranchVect->at(ibranch);
        if (!isEP(ibranch))  pbranchVect->at(ibranch).Ct = 0;
        else pbranchVect->at(ibranch).Ct = Cb;
    }
    for (it=index.begin(); it!=index.end();++it){
        ibranch = *it;
        branch = pbranchVect->at(ibranch);
        e = Edge (branch.p, pbranchVect->at(branch.n).p);
        distance = e.length();
        cap = distance*CUNIT;
        if (isEP(ibranch))//EP should add buffer cap
            cap += branch.Ct;
        while (branch.i != root){
            pbranchVect->at(branch.n).Ct += cap;
            branch = pbranchVect->at(branch.n);
        }

    }
}
void Tree_::CalculateElmore(){//Calculate Elmore values for all EP and MP. EP and MP may have different size due to eliminating MP
    for (int j = 0; j<MP.size();++j){
        int k = MP[j].first;
        MP[j].second = elmore(k);
    }
    for (int j = 0; j<EP.size();++j){
        int k = EP[j].first;
        EP[j].second = elmore(k);
    }
}
float Tree_::elmore (int ibranch){
    Branch branch;
    DTYPE distance;
    Edge e;

    float delay=0;
    int root = getRoot();
    while (ibranch != root){
        branch = pbranchVect->at(ibranch);
        e = Edge (branch.p, pbranchVect->at(branch.n).p);
        distance = e.c2_ - e.c1_;
        delay += RUNIT*distance*(0.5*CUNIT*distance+branch.Ct);
        ibranch = branch.n;
    }
    //delay+= pbranchVect->at(root).Rs*pbranchVect->at(root).Db;
    delay+= Rs*pbranchVect->at(root).Ct+Db;
    return delay; 
}
int Tree_::getStep(int ibranch) {//first turn in the path from ibranch to root
    Edge e1, e2;
    if (IsJoggingLeg(ibranch)){//if it is jogging leg, it has parent
        Branch branch1 = pbranchVect->at(ibranch);
        Branch branch2 = pbranchVect->at(branch1.n);
        e1 = Edge (branch1.p, branch2.p);
        e2 = Edge (branch2.p, pbranchVect->at(branch2.n).p);
    }
    else {
        int ibranch2 = ibranch, ibranchOnMaxSeg = firstParentOnMaxSeg (ibranch);
        if (ibranch2 == ibranchOnMaxSeg){//ibranch is on max segment
            if ( getOutLength(1, ibranch, root, getEdge(ibranch), getEdge(root)) < getOutLength(-1, ibranch, root, getEdge(ibranch), getEdge(root)) )
                return 1;
            return -1;
        }
        e2 = Edge (pbranchVect->at(ibranchOnMaxSeg).p, pbranchVect->at(pbranchVect->at(ibranchOnMaxSeg).n).p);// e2 is the edge on max segment
        while (pbranchVect->at(ibranch2).n != ibranchOnMaxSeg){
            ibranch2 = pbranchVect->at(ibranch2).n;
        }
        e1 = Edge (pbranchVect->at(ibranch2).p, pbranchVect->at(ibranchOnMaxSeg).p);//e1 is the edge before max segment
    }
    if (e1.v_&&e2.c_==e1.c1_&&e2.c1_==e1.c_)//down,right
        return 1;
    else if (e1.v_&&e2.c_==e1.c2_&&e2.c2_==e1.c_)//up,left
        return 1;
    else if (!e1.v_&&e2.c_==e1.c2_&&e2.c1_==e1.c_)//right,up
        return 1;
    else if (!e1.v_&&e2.c_==e1.c1_&&e2.c2_==e1.c_)//left,down
        return 1;
    else if (!e1.v_&&e2.c_==e1.c1_&&e2.c1_==e1.c_)//left,up
        return -1;
    else if (!e1.v_&&e2.c_==e1.c2_&&e2.c2_==e1.c_)//right,down
        return -1;
    else if (e1.v_&&e2.c_==e1.c2_&&e2.c1_==e1.c_)//up,right
        return -1;
    else if (e1.v_&&e2.c_==e1.c1_&&e2.c2_==e1.c_)//down,left
        return -1;
    else{
        if ( getOutLength(1, ibranch, root, getEdge(ibranch), getEdge(root)) < getOutLength(-1, ibranch, root, getEdge(ibranch), getEdge(root)) )
            return 1;
        return -1;
    }
}
int Tree_::getEdge(int ibranch)const{//return the start point of the edge
    int i;
    Edge blEdge;
    Point p = pbranchVect->at(ibranch).p;
    Poly block = pblockVect->at(iblock);
    for (i = 0; i < block.size(); ++i){
        blEdge = block.edge(i); 
        if (p.IsOnEdge(blEdge))
            return i;
    }
    return -1;//that's not gonna happen
}
int Tree_::findNextBranch (int ibranch, int step)const{//find edge numbers first. Find one with minimum edge number difference. If two with same gap, choose the one closer to branch
    vector<pair<int,float> >::const_iterator it;
    int iEdge,iEdge2;
    Poly block = pblockVect->at(iblock);
    int gap = block.size(), currentgap;//maxium+1 edge difference
    int ibranch2, iNearestBranch;
    Point p;
    iEdge = getEdge(ibranch);

    for (it = EP.begin(); it != EP.end(); ++it){
        ibranch2 = it->first;
        if (ibranch2==ibranch) //if ibranch2 is EP itself, continue, but we use it for root(not in EP)
            ibranch2 = root;
        iEdge2 = getEdge(ibranch2);
        currentgap = (iEdge2-iEdge) / step;
        if (currentgap==0){// on the same edge
            p = (block.p1_edge(iEdge));//the start point of this edge if step==1, otherwise it's end point
            if ((p.distance(pbranchVect->at(ibranch).p) < p.distance(pbranchVect->at(ibranch2).p) &&step==-1) || (p.distance(pbranchVect->at(ibranch).p) > p.distance(pbranchVect->at(ibranch2).p) &&step==1))
            {
                currentgap +=block.size();                
            }
        }else if (currentgap < 0) currentgap+=block.size();

        if (currentgap<gap){
            gap = currentgap;
            iNearestBranch = ibranch2;
        }else if (currentgap==gap){
            p = (block.p1_edge(iEdge2));
            if ((p.distance(pbranchVect->at(ibranch2).p) < p.distance(pbranchVect->at(iNearestBranch).p) &&step==1) || (p.distance(pbranchVect->at(ibranch2).p) > p.distance(pbranchVect->at(iNearestBranch).p) &&step==-1))
            {
                iNearestBranch = ibranch2;
            }

        }

    }
    return iNearestBranch;

}
int Tree_::FindFirstSteinerBranch(int ibranch) const{//find first steiner point (ibranch,root]. Will return -1 if no such point
    if (ibranch == root)    return -1;
    int index_branch = pbranchVect->at(ibranch).n;
    while (index_branch != root){
        if (CountChildren(index_branch) > 1)   return index_branch;
        index_branch = pbranchVect->at(index_branch).n;
    }
    return -1;
}
int Tree_::FindFirstCommonAncestor(int ibranch1, int ibranch2) const{//find first common ancestor of ibranch1 and ibranch2
    while (!IsAncestorBranch(ibranch1, ibranch2) && !IsAncestorBranch(ibranch2, ibranch1)){
       if (ibranch1 == root || ibranch2 == root)
           break;
       ibranch1 = pbranchVect->at(ibranch1).n;
       ibranch2 = pbranchVect->at(ibranch2).n;
    }
    if (ibranch1 == root || ibranch2 == root)   return root;
    if (IsAncestorBranch(ibranch1, ibranch2))
        return ibranch2;
    else 
        return ibranch1;
}
bool Tree_::IsAncestorBranch(int index_branch, int index_ancestor_branch)const{//a branch is considered as ancestor of itself
    while (index_branch != root){//use this instead of root to judge is because tree has lots of individual small trees which rooted themselves
        if (index_branch == index_ancestor_branch)   return true;
        index_branch = pbranchVect->at(index_branch).n;
    }
    return false;
}


bool Design::updateTree (int itree, const PRS & prs, vector<pair<int,int> >& unionR_i){//if some roots changed, these trees will only use mazerouting
    treeVect[itree].erasePoint(treeVect[itree].MP, prs.iworstBranch);//in this function need test if it is already erased

    int i, j, k, ibranch, iparentBranch, iworstBranch,ibranchOnMaxSeg, iEdge;
    Branch newTerminalBranch, newTerminalBranch2, newBranch;
    for (i=0; i < prs.size(); i++){//process each EP of tree i
        PR pri = prs.PRV[i];
        ibranch = pri.ibranch;
        if (pri.coefA[0]==1){//this MP combined
            treeVect[itree].erasePoint(treeVect[itree].MP, ibranch);
            treeVect[itree].erasePoint(treeVect[itree].EP, ibranch);
            treeVect[itree].eraseIndex(ibranch);//from ibranch to ibranchOnMaxSeg if not shared by another branch
            //if (!keyHas(unionR_i, ibranch))
            unionR_i.push_back(pair<int, int>(ibranch, itree) );
#ifdef INFO
            printf ("Branch%d is combined\n", ibranch);
#endif
            iparentBranch = branchVect[ibranch].n;
            treeVect[itree].CleanFloatingTerminal (iparentBranch);
            continue;//combined, no need to move
        }
        for (j = 1; j < pri.coefA.size(); ++j){//looking for which candidator of this EP is chosen
            if (pri.coefA[j] == 1){
                if (j==1 && pri.coefY[j] < EPSILON) continue;//not move due to calculation it may have a very small move
                Point p1 = pri.pstart[j];
                Point p2 = pri.pend[j];
                Point new_point;
                Edge e =  Edge (p1,p2);
                if (e.length()!=0){
                    new_point.x = p1.x + (p2.x-p1.x)*pri.coefY[j]/e.length();
                    new_point.y = p1.y + (p2.y-p1.y)*pri.coefY[j]/e.length();
                }else new_point = p1;
                Point new_EP = new_point;
                new_point = new_point.projection(treeVect[itree].ms);//yilin 2/28 has problem here when run RC11 with Slew : 30650. Because of leg structure project on its own instead of ms, will take care later
                newTerminalBranch = Branch(branchVect.size(), branchVect[ibranch].p, ibranch);//newTerminalBranch is a bridge from old EP to new EP
                for (k=0; k<branchVect.size();k++){//redirect to newTerminalBranch
                    if (branchVect[k].n == ibranch)
                        branchVect[k].n = newTerminalBranch.i;
                }
                branchVect.push_back(newTerminalBranch);
                tree.insertIndex(newTerminalBranch);//ibranch will be swaped with newTerminalBranch later
                ibranchOnMaxSeg = findBranch (new_point);
                if (ibranchOnMaxSeg != treeVect[itree].getRoot()){
                    newBranch = Branch(branchVect.size(), new_point, branchVect[ibranchOnMaxSeg].n);
                    branchVect[ibranchOnMaxSeg].n = newBranch.i;
                    branchVect[ibranch].p = new_EP;
                    branchVect[ibranch].n = newBranch.i;
                    branchVect.push_back(newBranch);
                    treeVect[itree].insertIndex(newBranch);
                }else{
                    branchVect[ibranch].p = new_EP;
                    branchVect[ibranch].n = treeVect[itree].getRoot();
                }
                if (!newTerminalBranch.p.PointsOnSameLine(branchVect[ibranch].p)){//insert a corner if ibranch and newTerminalBranch is not at same line, because other EP in this function will check
                    Point corner (branchVect[ibranch].p.x, newTerminalBranch.p.y); 
                    newTerminalBranch2 = Branch(branchVect.size(), corner, ibranch);
                    branchVect.push_back(newTerminalBranch2);
                    tree.insertIndex(newTerminalBranch2);//ibranch will be swaped with newTerminalBranch later
                    branchVect[newTerminalBranch.i].n = newTerminalBranch2.i;
                }
                if (ibranchOnMaxSeg != ibranch)//if ibranch and root are on a two EP line, ibranch is ibranchOnMaxSeg then
                    treeVect[itree].CleanFloatingTerminal (ibranchOnMaxSeg);//remove potential floating segment
                //if (!keyHas(unionR_i, ibranch))//This time ibranch is the index of branch inserted to unionR_i
                unionR_i.push_back(pair<int,int> (ibranch, itree) );
#ifdef INFO
                printf ("Branch%d is moving to position (%f,%f)\n", ibranch, (float)branchVect[ibranch].p.x, (float)branchVect[ibranch].p.y);
#endif
                break;//if  == 1, then no need to check others
            }
        }
    }
    //checkStraight();
}
void Tree_::buildPR ( PRS & prs){
    int i, j, k, ibranch, inextBranch,irootOnMaxSeg, ibranchOnMaxSeg, iworstBranchOnMaxSeg, current_edge, start_edge, stop_edge, step;//step = 1 or -1
    int iworstBranch = prs.iworstBranch;
    Poly block = pblockVect->at(iblock); //number of edges in the poly
    int blocksize = block.size(); //number of edges in the poly
    float fbase=0, fslope=0, fbase2, fslope2; 
    float fCa, fCc;//fCa is the downstream cap beside the ibranch path, fCb is the cap to root, fCc is cap on ibranch path to MaxSeg.
    float fRb, fRc;
    DTYPE dlength=0, dtotalLength, dtotalLength2, dWLbase, dWLbase2;
    Branch branch; 
    Point prootOnMaxSeg, pbranchOnMaxSeg, pworstBranchOnMaxSeg, pneighborBranchOnMaxSeg;
    Point pstart, pend;
    Edge edge, branchEdge;
    bool save;//save this for ibranch and iworstBranch or not
    bool bnewEdge, bendOfMaxSeg, brootBreak;

    for (j=0; j<MP.size(); ++j){//choosing move EP,start from the worst one
        ibranch = MP[j].first;  branch = pbranchVect->at(ibranch);
        step = getStep (ibranch);//branch and root
        inextBranch = findNextBranch (ibranch, step);
        start_edge = getEdge ( ibranch);     current_edge = start_edge;
        stop_edge = getEdge ( inextBranch);
        PR pr(ibranch, step);
        if (IsJoggingLeg(ibranch)){
            step = getStep(ibranch);
            inextBranch = findNextBranch (ibranch, step);
            start_edge = getEdge ( ibranch);
            stop_edge = getEdge ( inextBranch);
            PR pr(ibranch, step);
            addCombinationPR (prs, pr, ibranch, inextBranch, iworstBranch, ibranchOnMaxSeg, 0, start_edge, stop_edge);//bendOfMaxSeg is 0 for jogging leg
            continue;
        }


        irootOnMaxSeg = rootOnMaxSeg ();//these six lines for move range
        ibranchOnMaxSeg = firstParentOnMaxSeg (ibranch);
        iworstBranchOnMaxSeg = firstParentOnMaxSeg (iworstBranch);
        prootOnMaxSeg = pbranchVect->at(irootOnMaxSeg).p;
        pbranchOnMaxSeg = pbranchVect->at(ibranchOnMaxSeg).p;
        pworstBranchOnMaxSeg = pbranchVect->at(iworstBranchOnMaxSeg).p;
        edge = Edge (prootOnMaxSeg, pworstBranchOnMaxSeg); 
        if (!pbranchOnMaxSeg.IsOnEdge(edge) && pbranchOnMaxSeg!=prootOnMaxSeg)//branch is not on path (root, worst branch]
            continue;

        bendOfMaxSeg = (ms.point1()== pbranchOnMaxSeg) || (ms.point2()==pbranchOnMaxSeg);
        addCombinationPR (prs, pr, ibranch, inextBranch, iworstBranch, ibranchOnMaxSeg, bendOfMaxSeg, start_edge, stop_edge);

        bnewEdge = true;
        while (current_edge != (stop_edge+step+blocksize)%blocksize){
            save = true;
            brootBreak = false;
            /*---------------------- pstart, pend and some initial condition-------------*/
            if (bnewEdge){//a new edge
                if (step==1){
                    pstart = block.p1_edge(current_edge);
                    pend = block.p2_edge(current_edge);
                } else {
                    pstart = block.p2_edge(current_edge);
                    pend = block.p1_edge(current_edge);
                }
                if (current_edge == start_edge){
                    pstart = pbranchVect->at(ibranch).p;
                    fRb = getLength(ibranchOnMaxSeg, root)*RUNIT + Rs;//not to irootOnMaxSeg
                    fRc = getLength(ibranch, ibranchOnMaxSeg)*RUNIT;
                    fCc = getLength(ibranch, ibranchOnMaxSeg)*CUNIT + pbranchVect->at(ibranch).Ct;
                    fCa = pbranchVect->at(ibranchOnMaxSeg).Ct - fCc;
                    fbase2 = 0;
                    dWLbase2 = 0;
                }else if (current_edge == stop_edge){
                    break;//combine is better than connection with stopEnd
                    //pend =pbranchVect->at(inextBranch).p;
                }
            }else{//if same edge, pstart and pend were updated
                bnewEdge = true; //reset to default 
            }
            edge = Edge (pstart, pend);// edge is the line between start and end point
            dlength = edge.length();

            /*---------------------- take care of over root edge -------------*/
            if ( (prootOnMaxSeg.isProjectedIn(edge) && edge.v_==ms.v_ ) || ( (edge.c_==(edge.v_?prootOnMaxSeg.x:prootOnMaxSeg.y)) && edge.v_!=ms.v_ ) && !prootOnMaxSeg.IsOnEdge(edge)){//edge parallel to ms, root in edge. Or perpendicular, same line, but prootOnMaxSeg not on edge
                if (edge.v_==ms.v_){
                    pend =  prootOnMaxSeg.projection(edge);
                }
                brootBreak = true;
            }

            /*---------------------- calculation of current edge  -------------*/
            fbase = fbase2;
            dWLbase = dWLbase2;
            if (edge.v_ == ms.v_){//parallel
                if (ibranch==iworstBranch){//ibranch is iworst

                    if (bendOfMaxSeg){//ibranch(iworst) is at the end
                        pneighborBranchOnMaxSeg = pbranchVect->at(pbranchVect->at(ibranchOnMaxSeg).n).p;
                        Point ptemp;
                        if (pneighborBranchOnMaxSeg.isProjectedIn(edge)){
                            ptemp = pend;
                            pend = pneighborBranchOnMaxSeg.projection(edge);
                            addNonlinearPR (fbase, fbase2, dWLbase, dWLbase2, fCa, fRb, fRc, fCc, pstart, pend, branch, pr, bendOfMaxSeg, prs.deltaX);
                            pstart = pend;
                            pend = ptemp;
                            bnewEdge = false;
                        }else
                            addNonlinearPR (fbase, fbase2, dWLbase, dWLbase2, fCa, fRb, fRc, fCc, pstart, pend, branch, pr, bendOfMaxSeg, prs.deltaX);
                    }else
                        addNonlinearPR (fbase, fbase2, dWLbase, dWLbase2, fCa, fRb, fRc, fCc, pstart, pend, branch, pr, bendOfMaxSeg, prs.deltaX);

                }else{
                    fbase2 = fbase - fRb*dlength*CUNIT;
                    dWLbase2 = dWLbase + dlength;
                    fCa += dlength*CUNIT;//fCc and fres not chage
                    fRb -= dlength*RUNIT; 
                    pr.add (fbase,-fCc*RUNIT,dlength,dWLbase, (fRb-Rs)/RUNIT, bendOfMaxSeg, pstart,pend);
#ifdef DEBUG
                    printf ("Target branch is %d, branch%d on edge%d, base is %f, WLbase is %f\n",iworstBranch, ibranch, current_edge, fbase, (float)dWLbase);
#endif
                }
            }else{//perpendicular
                if (pstart.distance(ms) < pend.distance(ms)){//go away from ms
                    if (ibranch == iworstBranch)
                        fbase2 = fbase + 0.5*RUNIT*CUNIT*dlength*dlength + branch.Ct*dlength*RUNIT + dlength*CUNIT*(fRb+fRc) ;
                    else
                        fbase2 =  fbase + dlength*CUNIT*fRb;
                    fCc += CUNIT*dlength;
                    fRc += RUNIT*dlength;//fCa and fRb not change
                    dWLbase2 = dWLbase + 2*dlength;
                }else {
                    if (ibranch == iworstBranch)
                        fbase2 =  fbase - 0.5*RUNIT*CUNIT*dlength*dlength - branch.Ct*dlength*RUNIT - dlength*CUNIT*(fRb+fRc-dlength*RUNIT);
                    else
                        fbase2 =  fbase - dlength*CUNIT*fRb;
                    fbase = fbase2;
                    dlength = 0;//only record the point
                    pr.add (fbase,0,dlength,dWLbase, (fRb-Rs)/RUNIT, bendOfMaxSeg, pend,pend);//fslope is no care when dlength
#ifdef DEBUG
                    printf ("Target branch is %d, branch%d on edge%d, base is %f, WLbase is %f\n",iworstBranch, ibranch, current_edge, fbase, (float)dWLbase);
#endif
                    dWLbase2 = dWLbase;//no penalty if it is getting closer
                }
            }


            if (brootBreak)  break;
#ifdef DEBUG
            pr.show();
#endif
            if (bnewEdge)
                current_edge = (current_edge+step+blocksize)%blocksize;
        }//end of while
        // DTYPE djoinWLbase = findCloserNeighbor(ibranch);
        if (ibranch == iworstBranch){
            //  float fjoinBase = -1*EP[ibranch]
        }else {//if it's backtracing, make sure it's no worth than spec
            //...
        }
        prs.add(pr);
    }//end of any moving point
}
void PR::show () const{//draw the possible regions for one MP
    FILE * fp;
    string str= "/home/polaris/yzhang1/scripts/pr_plot_";
    stringstream ss;
    ss << iworstBranch << "_" << ibranch<< ".gpl";
    str = str + ss.str();
    int i;
    DTYPE xstart, xend;
    float ystart, yend;
    DTYPE xleft=INT_MAX, xright=INT_MIN;
    float ybottom=FLT_MAX, ytop=-FLT_MAX;//FLT_MIN is positive
    if( (fp=fopen(str.c_str(),"w")) == NULL ) {
        sprintf(error_text, "bookshelf_IO: Cannot open: %s file for write", str.c_str());
        runtimeError(error_text);
    }
    for (i=0; i<base.size(); ++i){
        xstart = WLbase[i];
        xend = xstart + length[i];
        ystart = base[i];
        yend = base[i] + slope[i]*length[i];
        if (min(xstart,xend)<xleft)   xleft = min(xstart,xend);
        if (max(xstart,xend)>xright)    xright = max(xstart,xend);
        if (min(ystart,yend)<ybottom) ybottom = min(ystart,yend);
        if (max(ystart,yend)>ytop)  ytop = max(ystart,yend);
        fprintf (fp, "set arrow from %f,%f to %f,%f nohead\n", (float)xstart,ystart,(float)xend,yend);
    }
    fprintf (fp, "set xrange [ %f : %f ]\n", (float)xleft, (float)xright);
    fprintf (fp, "set yrange [ %f : %f ]\n", ybottom, ytop);
    fprintf (fp, "plot x with dots");
    fclose(fp);
    //    remove(tempStr);
}
void PRS::write() const{
    FILE * fp;
    string str= "/home/polaris/yzhang1/Implementation/flute-3.1/flute-3.1/tmp/problem.lp";
    if( (fp=fopen(str.c_str(),"w")) == NULL ) {
        sprintf(error_text, "bookshelf_IO: Cannot open: %s file for write", str.c_str());
        runtimeError(error_text);
    }

    fprintf (fp, "Minimize\n");
    for (int i=0; i<PRV.size(); ++i){//objective Min. sum(sum(Yij+Aij*Bij-ALPHA*Aij*(ShareMaxSegij-Xij)))
        PR pri = PRV[i];
        for (int j=0; j<pri.size(); j++){
            if (pri.length[j]>0 && !pri.endOfMaxSeg[j]){//if it's end, then no penalty on moving
                if (  !(i==0 && j==0) ) fprintf (fp, " + ");
                fprintf (fp, "Y_%d_%d", i, j);
            }
            if (pri.WLbase[j]>0){//if length(dWLbase)==0, no need of Aij*Bij
                if (  !(i==0 && j==0 && pri.length[j]==0) ) fprintf (fp, " + ");
                fprintf (fp, "%f A_%d_%d", (float)pri.WLbase[j], i, j);
            }
            if (pri.shareMaxSeg[j]>0){
                if ( !(i==0 && j==0) )  fprintf (fp, " - ");  
                fprintf (fp, "%f A_%d_%d + %f Y_%d_%d", ALPHA*pri.shareMaxSeg[j], i, j, ALPHA, i, j);
            }
            if (i==PRV.size()-1 && j==pri.size()-1)
                fprintf (fp, "\n");
        }
    } 
    fprintf (fp, "Subject To\n");
    for (int i=0; i<PRV.size(); ++i){//slew
        PR pri = PRV[i];
        for (int j=0; j<pri.size(); j++){
            if (pri.length[j]>0){
                if (  !(i==0 && j==0) ) fprintf (fp, " + ");
                fprintf (fp, "%f Y_%d_%d", pri.slope[j], i, j);
            }
            if (  !(i==0 && j==0 && pri.length[j]==0) ) fprintf (fp, " + ");
            fprintf (fp, "%f A_%d_%d", pri.base[j], i, j);
            if (  i==PRV.size()-1 && j==pri.size()-1)
                fprintf (fp, " <= %f\n", slew_spec_-slew0);
        }
    }
    for (int i=0; i<PRV.size(); ++i){//sum(Aij)==1 => sum(Aij)<=1, without the orignal position
        PR pri = PRV[i];
        for (int j=0; j<pri.size(); j++){
            if ( j!=0 ) fprintf (fp, " + ");
            fprintf (fp, "A_%d_%d", i, j);
            if ( j==pri.size()-1 ) fprintf (fp, " <= 1\n");
        }
    }
    for (int i=0; i<PRV.size(); ++i){//Yij -Aij*Xij_H  <= 0(0:Yij==0,1:Yij<=Xij_H)
        PR pri = PRV[i];
        for (int j=0; j<pri.size(); j++){
            if (pri.length[j]>0)
                fprintf (fp, "Y_%d_%d  - %f A_%d_%d <= 0\n", i, j, (float)pri.length[j], i, j);
        }
    }
    /*    for (int i=0; i<PRV.size(); ++i){//Yij  - Aij*Xij_L >= 0(0:Yij>=0, 1:Yij>=Xij_L)
          PR pri = PRV[i];
          for (int j=0; j<pri.size(); j++){
          fprintf (fp, "Y_%d_%d  - A_%d_%d %f  >= 0\n", i, j, i, j, 0);
          }
          }no need because Yij>=0*/
    fprintf (fp, "Binaries\n");
    for (int i=0; i<PRV.size(); ++i){//Aij binaries
        PR pri = PRV[i];
        for (int j=0; j<pri.size(); j++){
            fprintf (fp, "A_%d_%d", i, j);
            if (  i==PRV.size()-1 && j==pri.size()-1)   fprintf (fp, " ");
            else    fprintf (fp, "\n");
        }
    }
    fclose(fp);
}
void PRS::read(){
    FILE * fp;
    string str= "/home/polaris/yzhang1/Implementation/flute-3.1/flute-3.1/tmp/problem.sol";
    if( (fp=fopen(str.c_str(),"r")) == NULL ) {
        sprintf(error_text, "bookshelf_IO: Cannot open: %s file for write", str.c_str());
        runtimeError(error_text);
    }
    int i, j;
    float value;
    char temp[BUFFERSIZE];
    char variableName;
    char line[LINESIZE];
    char *pch = NULL;
    while(!feof(fp)) {
        *line = '\0';
        fgets(line, LINESIZE, fp);
        sscanf(line, "%s\t%*s\n", temp);
        if( strlen(line)<5 || temp[0] == '#')
            continue;

        sscanf(line, "%s\t%f\n", temp, &value);
        pch = strtok (temp, "_");
        variableName = *pch;
        pch = strtok (NULL, "_");
        i = *pch - '0';
        pch = strtok (NULL, "_");
        j = *pch - '0';
        if (variableName == 'A')
            PRV[i].coefA[j] = (int)value;
        else if (variableName == 'Y')
            PRV[i].coefY[j] = value;
    }
    fclose(fp);
}
