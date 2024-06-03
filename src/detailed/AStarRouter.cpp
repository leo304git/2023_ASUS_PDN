#include "AStarRouter.h"

bool AStarRouter::route() {
    // define the priority queue
    auto cmp = [](GNode* a, GNode* b) { return a->cost() < b->cost(); };
    multiset<GNode*, decltype(cmp) > Q(cmp);

    // initialize the source node
    GNode* sNode = _vGNode[_sPos.first][_sPos.second];
    sNode->setParent(sNode);
    sNode->setCost(0.0);
    sNode->setCurDist(0.0);
    sNode->setEstDist(0.0);
    sNode->setCongest(0.0);
    Q.insert(sNode);
    sNode->setStatus(GNodeStatus::InQueue);

    // define lambda functions
    auto step = [&](GNode* orgNode, int xId, int yId, Direction dir) -> bool {
        GNode* stepNode = _vGNode[xId][yId];
        double newCurDist, newEstDist, newLineDist, newCongest, newCost;
        if (xId == _tPos.first && yId == _tPos.second) {
        // if ((dir == Direction::Up && xId == _tPos.first && yId + floor(ceil(_lbWidth/_gridWidth)/2.0) == _tPos.second) ||
        //     (dir == Direction::Down && xId == _tPos.first && yId - floor(ceil(_lbWidth/_gridWidth)/2.0) == _tPos.second) ||
        //     (dir == Direction::Right && xId + floor(ceil(_lbWidth/_gridWidth)/2.0) == _tPos.first && yId == _tPos.second) ||
        //     (dir == Direction::Left && xId - floor(ceil(_lbWidth/_gridWidth)/2.0) == _tPos.first && yId == _tPos.second)) {
            stepNode->setParent(orgNode);
            backTraceNoPad();
            // backTrace(xId, yId);
            // cerr << _vGNode[-1][-1] << endl;
            return true;
        } else if (stepNode->status() != GNodeStatus::InPath) {
            // cerr << "stepNode = (" << stepNode->xId() << ", " << stepNode->yId() << "), address = " << stepNode;
            // cerr << ", cost = " << stepNode->cost() << ;
            newCurDist = orgNode->curDist()+1;
            newEstDist = estDistCost(xId, yId);
            newLineDist = lineDistCost(xId, yId);
            // cerr << ", lineDist = " << newLineDist;
            newCongest = marginCongestCost(xId, yId, dir) + orgNode->congest();
            // cerr << ", marginCongestCost = " << marginCongestCost(xId, yId, dir);
            newCost = _distWeight*(newCurDist + newEstDist) + _cLineDistWeight*newLineDist + (1-_distWeight-_cLineDistWeight)*newCongest;
            if (stepNode->status() == GNodeStatus::Init) {
                // cerr << stepNode << ": Init" << endl;
                stepNode->setCurDist(newCurDist);
                stepNode->setEstDist(newEstDist);
                stepNode->setCongest(newCongest);
                stepNode->setCost(newCost);
                stepNode->setParent(orgNode);
                stepNode->setStatus(GNodeStatus::InQueue);
                Q.insert(stepNode);
            } else {
                assert(stepNode->status() == GNodeStatus::InQueue);
                if (stepNode->cost() > newCost) {
                    // cerr << stepNode << ": InQueue" << endl;
                    // Q.erase(Q.find(stepNode));
                    stepNode->setCurDist(newCurDist);
                    stepNode->setEstDist(newEstDist);
                    stepNode->setCongest(newCongest);
                    stepNode->setCost(newCost);
                    stepNode->setParent(orgNode);
                    Q.insert(stepNode);
                }
            }
            // cerr << "stepNode = (" << stepNode->xId() << ", " << stepNode->yId() << "), cost = " << stepNode->cost() << ", address = " << stepNode << endl;
            // cerr << ", cost = " << stepNode->cost() << endl;
            return false;
        }
        return false;
    };

    // start searching
    while (!Q.empty()) {
        GNode* node = *Q.begin();
        Q.erase(Q.begin());
        // cerr << "node = (" << node->xId() << ", " << node->yId() << "), ";
        // cerr << "node->cost() = " << node->cost() << endl;
        node->setStatus(GNodeStatus::InPath);
        if (legal(node->xId(), node->yId()+1)) {
            if (step(node, node->xId(), node->yId()+1, Direction::Up)) return true;
        }
        if (legal(node->xId(), node->yId()-1)) {
            if (step(node, node->xId(), node->yId()-1, Direction::Down)) return true;
        }
        if (legal(node->xId()+1, node->yId())) {
            if (step(node, node->xId()+1, node->yId(), Direction::Right)) return true;
        }
        if (legal(node->xId()-1, node->yId())) {
            if (step(node, node->xId()-1, node->yId(), Direction::Left)) return true;
        }
        if (legal(node->xId()+1, node->yId()+1)) {
            if (step(node, node->xId()+1, node->yId()+1, Direction::UpRight)) return true;
        }
        if (legal(node->xId()-1, node->yId()+1)) {
            if (step(node, node->xId()-1, node->yId()+1, Direction::UpLeft)) return true;
        }
        if (legal(node->xId()+1, node->yId()-1)) {
            if (step(node, node->xId()+1, node->yId()-1, Direction::DownRight)) return true;
        }
        if (legal(node->xId()-1, node->yId()-1)) {
            if (step(node, node->xId()-1, node->yId()-1, Direction::DownLeft)) return true;
        }
        // if (node->yId() < numYId()-1-floor(ceil(_lbWidth)/2.0)) {
        //     if (step(node, node->xId(), node->yId()+1, Direction::Up)) return true;
        // }
        // if (node->yId() > 0+floor(ceil(_lbWidth)/2.0)) {
        //     if (step(node, node->xId(), node->yId()-1, Direction::Down)) return true;
        // }
        // if (node->xId() < numXId()-1-floor(ceil(_lbWidth)/2.0)) {
        //     if (step(node, node->xId()+1, node->yId(), Direction::Right)) return true;
        // }
        // if (node->xId() > 0+floor(ceil(_lbWidth)/2.0)) {
        //     if (step(node, node->xId()-1, node->yId(), Direction::Left)) return true;
        // }
        // cerr << "Q = " << endl;
        // for (set<GNode*, decltype(cmp) >::iterator i=Q.begin(); i!=Q.end(); ++i) {
        //     cerr << "(" << *i << ", " << *i << ")" << endl; 
        // }
        // vector<Grid*> gg;
        // if (node->yId() == 58) cerr << gg[2] << endl;
    }
    return false;
}

void AStarRouter::backTrace(int tXId, int tYId) {
    // GNode* node = _vGNode[_tPos.first][_tPos.second];
    GNode* node = _vGNode[tXId][tYId];
    while(node->parent() != node) {
        _path.push_back(_vGrid[node->xId()][node->yId()]);
        node = node->parent();
    }
    assert(node->xId() == _sPos.first && node->yId() == _sPos.second);
    _path.push_back(_vGrid[node->xId()][node->yId()]);

    _exactLength = _path.size()-1;
    // double length = _exactLength * _gridWidth;
    // _exactWidth = ceil(_lbWidth * _exactLength / _lbLength);
    _exactWidth = ceil(_lbWidth/_gridWidth);
    // cerr << "_length = " << _lbLength << ", _exactLength = " << _exactLength << " _width = " << _lbWidth << ", _exactWidth = " << _exactWidth << endl;

    // around the ending grid
    for (int xId = _tPos.first - floor(_exactWidth/2.0); xId <= _tPos.first + floor(_exactWidth/2.0); ++ xId) {
        if (xId >= 0 && xId < numXId()) {
            for (int yId = _tPos.second - floor(_exactWidth/2.0); yId <= _tPos.second + floor(_exactWidth/2.0); ++ yId) {
                if (yId >= 0 && yId < numYId()) {
                    _vPGrid.push_back(_vGrid[xId][yId]);
                }
            }
        }
    }
    // around the path
    node = _vGNode[_tPos.first][_tPos.second];
    while(node->parent() != node) {
        // cerr << "node->parent() = (" << node->parent()->xId() << ", " << node->parent()->yId() << ")" << endl;
        // vector<Grid*> gg;
        // if (node->parent()->xId() == 21 && node->parent()->yId() ==  58) cerr << gg[2] << endl;
        if (node->parent()->xId() == node->xId()+1 && node->parent()->yId() == node->yId()) {   // Right
            for (int y = node->parent()->yId()-floor(_exactWidth/2.0); y <= node->parent()->yId()+floor(_exactWidth/2.0); ++ y) {
                if (legal(node->parent()->xId() + floor(_exactWidth/2.0), y)) {
                    _vPGrid.push_back(_vGrid[node->parent()->xId() + floor(_exactWidth/2.0)][y]);
                }
            }
            // if (node->parent()->xId() + floor(ceil(_lbWidth/_gridWidth)/2.0) == _sPos.first && node->parent()->yId() == _sPos.second) {
            //     break;
            // }
        } else if (node->parent()->xId() == node->xId()-1  && node->parent()->yId() == node->yId()) {   // Left
            for (int y = node->parent()->yId()-floor(_exactWidth/2.0); y <= node->parent()->yId()+floor(_exactWidth/2.0); ++ y) {
                if (legal(node->parent()->xId() - floor(_exactWidth/2.0), y)) {
                    _vPGrid.push_back(_vGrid[node->parent()->xId() - floor(_exactWidth/2.0)][y]);
                }
            }
            // if (node->parent()->xId() - floor(ceil(_lbWidth/_gridWidth)/2.0) == _sPos.first && node->parent()->yId() == _sPos.second) {
            //     break;
            // }
        } else if (node->parent()->xId() == node->xId() && node->parent()->yId() == node->yId()+1) {    // Up
            for (int x = node->parent()->xId()-floor(_exactWidth/2.0); x <= node->parent()->xId()+floor(_exactWidth/2.0); ++ x) {
                if (legal(x, node->parent()->yId() + floor(_exactWidth/2.0))) {
                    _vPGrid.push_back(_vGrid[x][node->parent()->yId() + floor(_exactWidth/2.0)]);
                }
            }
            // if (node->parent()->xId() == _sPos.first && node->parent()->yId() + floor(ceil(_lbWidth/_gridWidth)/2.0) == _sPos.second) {
            //     break;
            // }
        } else if (node->parent()->xId() == node->xId() && node->parent()->yId() == node->yId()-1) {    // Down
            for (int x = node->parent()->xId()-floor(_exactWidth/2.0); x <= node->parent()->xId()+floor(_exactWidth/2.0); ++ x) {
                if (legal(x, node->parent()->yId() - floor(_exactWidth/2.0))) {
                    _vPGrid.push_back(_vGrid[x][node->parent()->yId() - floor(_exactWidth/2.0)]);
                }
            }
            // if (node->parent()->xId() == _sPos.first && node->parent()->yId() - floor(ceil(_lbWidth/_gridWidth)/2.0) == _sPos.second) {
            //     break;
            // }
        } else if (node->parent()->xId() == node->xId()+1 && node->parent()->yId() == node->yId()+1) {  // UpRight
            for (int x = node->parent()->xId()-floor(_exactWidth/2.0); x <= node->parent()->xId()+floor(_exactWidth/2.0); ++ x) {
                if (legal(x, node->parent()->yId() + floor(_exactWidth/2.0))) {
                    _vPGrid.push_back(_vGrid[x][node->parent()->yId() + floor(_exactWidth/2.0)]);
                }
            }
            for (int y = node->parent()->yId()-floor(_exactWidth/2.0); y <= node->parent()->yId()+floor(_exactWidth/2.0)-1; ++ y) {
                if (legal(node->parent()->xId() + floor(_exactWidth/2.0), y)) {
                    _vPGrid.push_back(_vGrid[node->parent()->xId() + floor(_exactWidth/2.0)][y]);
                }
            }
        } else if (node->parent()->xId() == node->xId()-1 && node->parent()->yId() == node->yId()+1) {  // UpLeft
            for (int x = node->parent()->xId()-floor(_exactWidth/2.0); x <= node->parent()->xId()+floor(_exactWidth/2.0); ++ x) {
                if (legal(x, node->parent()->yId() + floor(_exactWidth/2.0))) {
                    _vPGrid.push_back(_vGrid[x][node->parent()->yId() + floor(_exactWidth/2.0)]);
                }
            }
            for (int y = node->parent()->yId()-floor(_exactWidth/2.0); y <= node->parent()->yId()+floor(_exactWidth/2.0)-1; ++ y) {
                if (legal(node->parent()->xId() - floor(_exactWidth/2.0), y)) {
                    _vPGrid.push_back(_vGrid[node->parent()->xId() - floor(_exactWidth/2.0)][y]);
                }
            }
        } else if (node->parent()->xId() == node->xId()+1 && node->parent()->yId() == node->yId()-1) {  // DownRight
            for (int x = node->parent()->xId()-floor(_exactWidth/2.0); x <= node->parent()->xId()+floor(_exactWidth/2.0); ++ x) {
                if (legal(x, node->parent()->yId() - floor(_exactWidth/2.0))) {
                    _vPGrid.push_back(_vGrid[x][node->parent()->yId() - floor(_exactWidth/2.0)]);
                }
            }
            for (int y = node->parent()->yId()-floor(_exactWidth/2.0)+1; y <= node->parent()->yId()+floor(_exactWidth/2.0); ++ y) {
                if (legal(node->parent()->xId() + floor(_exactWidth/2.0), y)) {
                    _vPGrid.push_back(_vGrid[node->parent()->xId() + floor(_exactWidth/2.0)][y]);
                }
            }
        } else {  // DownLeft
            assert(node->parent()->xId() == node->xId()-1 && node->parent()->yId() == node->yId()-1);
            for (int x = node->parent()->xId()-floor(_exactWidth/2.0); x <= node->parent()->xId()+floor(_exactWidth/2.0); ++ x) {
                if (legal(x, node->parent()->yId() - floor(_exactWidth/2.0))) {
                    _vPGrid.push_back(_vGrid[x][node->parent()->yId() - floor(_exactWidth/2.0)]);
                }
            }
            for (int y = node->parent()->yId()-floor(_exactWidth/2.0)+1; y <= node->parent()->yId()+floor(_exactWidth/2.0); ++ y) {
                if (legal(node->parent()->xId() - floor(_exactWidth/2.0), y)) {
                    _vPGrid.push_back(_vGrid[node->parent()->xId() - floor(_exactWidth/2.0)][y]);
                }
            }
        }
        node = node->parent();
    } 
}

void AStarRouter::backTraceNoPad() {
    auto encloseNode = [&] (int centerX, int centerY, int radius, int enclosedX, int enclosedY) -> bool {
        assert(legal(centerX, centerY));
        assert(legal(enclosedX, enclosedY));
        if (enclosedX >= centerX-radius && enclosedX <= centerX+radius &&
            enclosedY >= centerY-radius && enclosedY <= centerY+radius) {
                return true;
            } 
        return false;
    };
    GNode* node = _vGNode[_tPos.first][_tPos.second];
    // GNode* node = _vGNode[tXId][tYId];
    _exactLength = 0.0;
    while(node->parent() != node) {
        _path.push_back(_vGrid[node->xId()][node->yId()]);
        node = node->parent();
        if (node->parent()->xId() == node->xId() || node->parent()->yId() == node->yId()) {
            _exactLength += 1;
        } else {
            _exactLength += sqrt(2.0);
        }
    }
    assert(node->xId() == _sPos.first && node->yId() == _sPos.second);
    _path.push_back(_vGrid[node->xId()][node->yId()]);

    // _exactLength = _path.size()-1;
    // double length = _exactLength * _gridWidth;
    _exactWidth = ceil(_lbWidth * _exactLength / _lbLength);
    if (_exactWidth < ceil(_lbWidth/_gridWidth)) {
        _exactWidth = ceil(_lbWidth/_gridWidth);
        cerr << "WARNING: A* grid length < global length !" << endl;
    }
    // _exactWidth = ceil(_lbWidth/_gridWidth);
    int halfWidth = ceil(0.5 * _lbWidth / _gridWidth);
    // int halfWidth = ceil(0.5 * _exactWidth);
    // double lineLength = pathLength(3, 0) * _gridWidth;
    // cerr << "lbLength = " << _lbLength << ", _exactLength*gridWidth = " << _exactLength*_gridWidth << ", lineLength = " << lineLength <<  endl;
    // cerr << "lbWidth = " << _lbWidth << ", _exactWidth*gridWidth = " << _exactWidth*_gridWidth << endl;
    size_t sPathId = _path.size()-1;
    size_t tPathId = 0;
    bool TEncloseS = false;
    bool SEncloseT = false;
    assert(encloseNode(_path[sPathId]->xId(), _path[sPathId]->yId(), halfWidth, _sRealPos.first, _sRealPos.second));
    while(encloseNode(_path[sPathId]->xId(), _path[sPathId]->yId(), halfWidth, _sRealPos.first, _sRealPos.second)) {
        if (sPathId == 0) {
            TEncloseS == true;
            break;
        }
        sPathId --;
    }
    sPathId ++;
    assert(encloseNode(_path[sPathId]->xId(), _path[sPathId]->yId(), halfWidth, _sRealPos.first, _sRealPos.second));
    assert(encloseNode(_path[tPathId]->xId(), _path[tPathId]->yId(), halfWidth, _tRealPos.first, _tRealPos.second));
    while(encloseNode(_path[tPathId]->xId(), _path[tPathId]->yId(), halfWidth, _tRealPos.first, _tRealPos.second)) {
        if (tPathId == _path.size()-1) {
            SEncloseT = true;
            break;
        }
        tPathId ++;
    }
    tPathId --;
    assert(encloseNode(_path[tPathId]->xId(), _path[tPathId]->yId(), halfWidth, _tRealPos.first, _tRealPos.second));
    assert(TEncloseS == SEncloseT);

    if (TEncloseS) {
        size_t cPathId = _path.size() / 2;
        for (int xId = _path[cPathId]->xId() - halfWidth; xId <= _path[cPathId]->xId() + halfWidth; ++ xId) {
            if (xId >= 0 && xId < numXId()) {
                for (int yId = _path[cPathId]->yId() - halfWidth; yId <= _path[cPathId]->yId() + halfWidth; ++ yId) {
                    if (yId >= 0 && yId < numYId()) {
                        _vPGrid.push_back(_vGrid[xId][yId]);
                    }
                }
            }
        }
    } else {
        // around the ending grid
        for (int xId = _path[tPathId]->xId() - halfWidth; xId <= _path[tPathId]->xId() + halfWidth; ++ xId) {
            if (xId >= 0 && xId < numXId()) {
                for (int yId = _path[tPathId]->yId() - halfWidth; yId <= _path[tPathId]->yId() + halfWidth; ++ yId) {
                    if (yId >= 0 && yId < numYId()) {
                        _vPGrid.push_back(_vGrid[xId][yId]);
                    }
                }
            }
        }
        // around the path
        // node = _vGNode[_tPos.first][_tPos.second];
        // while(node->parent() != node) {
        for (size_t pathId = tPathId; pathId < sPathId; ++ pathId) {
            // cerr << "node->parent() = (" << node->parent()->xId() << ", " << node->parent()->yId() << ")" << endl;
            // vector<Grid*> gg;
            // if (node->parent()->xId() == 21 && node->parent()->yId() ==  58) cerr << gg[2] << endl;
            if (_path[pathId+1]->xId() == _path[pathId]->xId()+1 && _path[pathId+1]->yId() == _path[pathId]->yId()) {   // Right
                for (int y = _path[pathId+1]->yId()-halfWidth; y <= _path[pathId+1]->yId()+halfWidth; ++ y) {
                    if (legal(_path[pathId+1]->xId() + halfWidth, y)) {
                        _vPGrid.push_back(_vGrid[_path[pathId+1]->xId() + halfWidth][y]);
                    }
                }
                // if (node->parent()->xId() + floor(ceil(_lbWidth/_gridWidth)/2.0) == _sPos.first && node->parent()->yId() == _sPos.second) {
                //     break;
                // }
            } else if (_path[pathId+1]->xId() == _path[pathId]->xId()-1  && _path[pathId+1]->yId() == _path[pathId]->yId()) {   // Left
                for (int y = _path[pathId+1]->yId()-halfWidth; y <= _path[pathId+1]->yId()+halfWidth; ++ y) {
                    if (legal(_path[pathId+1]->xId() - halfWidth, y)) {
                        _vPGrid.push_back(_vGrid[_path[pathId+1]->xId() - halfWidth][y]);
                    }
                }
                // if (node->parent()->xId() - floor(ceil(_lbWidth/_gridWidth)/2.0) == _sPos.first && node->parent()->yId() == _sPos.second) {
                //     break;
                // }
            } else if (_path[pathId+1]->xId() == _path[pathId]->xId() && _path[pathId+1]->yId() == _path[pathId]->yId()+1) {    // Up
                for (int x = _path[pathId+1]->xId()-halfWidth; x <= _path[pathId+1]->xId()+halfWidth; ++ x) {
                    if (legal(x, _path[pathId+1]->yId() + halfWidth)) {
                        _vPGrid.push_back(_vGrid[x][_path[pathId+1]->yId() + halfWidth]);
                    }
                }
                // if (node->parent()->xId() == _sPos.first && node->parent()->yId() + floor(ceil(_lbWidth/_gridWidth)/2.0) == _sPos.second) {
                //     break;
                // }
            } else if (_path[pathId+1]->xId() == _path[pathId]->xId() && _path[pathId+1]->yId() == _path[pathId]->yId()-1) {    // Down
                for (int x = _path[pathId+1]->xId()-halfWidth; x <= _path[pathId+1]->xId()+halfWidth; ++ x) {
                    if (legal(x, _path[pathId+1]->yId() - halfWidth)) {
                        _vPGrid.push_back(_vGrid[x][_path[pathId+1]->yId() - halfWidth]);
                    }
                }
                // if (node->parent()->xId() == _sPos.first && node->parent()->yId() - floor(ceil(_lbWidth/_gridWidth)/2.0) == _sPos.second) {
                //     break;
                // }
            } else if (_path[pathId+1]->xId() == _path[pathId]->xId()+1 && _path[pathId+1]->yId() == _path[pathId]->yId()+1) {  // UpRight
                for (int x = _path[pathId+1]->xId()-halfWidth; x <= _path[pathId+1]->xId()+halfWidth; ++ x) {
                    if (legal(x, _path[pathId+1]->yId() + halfWidth)) {
                        _vPGrid.push_back(_vGrid[x][_path[pathId+1]->yId() + halfWidth]);
                    }
                }
                for (int y = _path[pathId+1]->yId()-halfWidth; y <= _path[pathId+1]->yId()+halfWidth-1; ++ y) {
                    if (legal(_path[pathId+1]->xId() + halfWidth, y)) {
                        _vPGrid.push_back(_vGrid[_path[pathId+1]->xId() + halfWidth][y]);
                    }
                }
            } else if (_path[pathId+1]->xId() == _path[pathId]->xId()-1 && _path[pathId+1]->yId() == _path[pathId]->yId()+1) {  // UpLeft
                for (int x = _path[pathId+1]->xId()-halfWidth; x <= _path[pathId+1]->xId()+halfWidth; ++ x) {
                    if (legal(x, _path[pathId+1]->yId() + halfWidth)) {
                        _vPGrid.push_back(_vGrid[x][_path[pathId+1]->yId() + halfWidth]);
                    }
                }
                for (int y = _path[pathId+1]->yId()-halfWidth; y <= _path[pathId+1]->yId()+halfWidth-1; ++ y) {
                    if (legal(_path[pathId+1]->xId() - halfWidth, y)) {
                        _vPGrid.push_back(_vGrid[_path[pathId+1]->xId() - halfWidth][y]);
                    }
                }
            } else if (_path[pathId+1]->xId() == _path[pathId]->xId()+1 && _path[pathId+1]->yId() == _path[pathId]->yId()-1) {  // DownRight
                for (int x = _path[pathId+1]->xId()-halfWidth; x <= _path[pathId+1]->xId()+halfWidth; ++ x) {
                    if (legal(x, _path[pathId+1]->yId() - halfWidth)) {
                        _vPGrid.push_back(_vGrid[x][_path[pathId+1]->yId() - halfWidth]);
                    }
                }
                for (int y = _path[pathId+1]->yId()-halfWidth+1; y <= _path[pathId+1]->yId()+halfWidth; ++ y) {
                    if (legal(_path[pathId+1]->xId() + halfWidth, y)) {
                        _vPGrid.push_back(_vGrid[_path[pathId+1]->xId() + halfWidth][y]);
                    }
                }
            } else {  // DownLeft
                assert(_path[pathId+1]->xId() == _path[pathId]->xId()-1 && _path[pathId+1]->yId() == _path[pathId]->yId()-1);
                for (int x = _path[pathId+1]->xId()-halfWidth; x <= _path[pathId+1]->xId()+halfWidth; ++ x) {
                    if (legal(x, _path[pathId+1]->yId() - halfWidth)) {
                        _vPGrid.push_back(_vGrid[x][_path[pathId+1]->yId() - halfWidth]);
                    }
                }
                for (int y = _path[pathId+1]->yId()-halfWidth+1; y <= _path[pathId+1]->yId()+halfWidth; ++ y) {
                    if (legal(_path[pathId+1]->xId() - halfWidth, y)) {
                        _vPGrid.push_back(_vGrid[_path[pathId+1]->xId() - halfWidth][y]);
                    }
                }
            }
            // node = node->parent();
        } 
    }
}

double AStarRouter::marginCongestCost(int xId, int yId, Direction dir) {
    auto prob = [&](int w) -> double{
        return pow(1-_widthRatio, w-ceil(_lbWidth/_gridWidth)) * _widthRatio;
    };
    double congestion = 0.0;
    double threshold = 0.0001;
    for (int w = ceil(_lbWidth/_gridWidth); prob(w) > threshold; ++ w) {
        // cerr << "w = " << w << ", prob = " << prob(w) << endl;
        double wCongestion = 0.0;
        if (dir == Direction::Up) {
            // cerr << "Up" << endl;
            for (int x = xId-floor(w/2.0); x <= xId+floor(w/2.0); ++ x) {
                // cerr << "x = " << x << ", y = " << yId+floor(w/2.0) << endl;
                if (legal(x, yId+floor(w/2.0))) {
                    wCongestion += _vGrid[x][yId+floor(w/2.0)]->congestion();
                } else {
                    wCongestion += _obsCongest;
                }
            }
        } else if (dir == Direction::Down) {
            // cerr << "Down" << endl;
            for (int x = xId-floor(w/2.0); x <= xId+floor(w/2.0); ++ x) {
                // cerr << "x = " << x << ", y = " << yId-floor(w/2.0) << endl;
                if (legal(x, yId-floor(w/2.0))) {
                    wCongestion += _vGrid[x][yId-floor(w/2.0)]->congestion();
                } else {
                    wCongestion += _obsCongest;
                }
            }
        } else if (dir == Direction::Right) {
            // cerr << "Right" << endl;
            for (int y = yId-floor(w/2.0); y <= yId+floor(w/2.0); ++ y) {
                // cerr << "x = " << xId+floor(w/2.0) << ", y = " << y << endl;
                if (legal(xId+floor(w/2.0), y)) {
                    wCongestion += _vGrid[xId+floor(w/2.0)][y]->congestion();
                } else {
                    wCongestion += _obsCongest;
                }
            }
        } else if (dir == Direction::Left) {
            // cerr << "Left" << endl;
            for (int y = yId-floor(w/2.0); y <= yId+floor(w/2.0); ++ y) {
                // cerr << "x = " << xId-floor(w/2.0) << ", y = " << y << endl;
                if (legal(xId-floor(w/2.0), y)) {
                    wCongestion += _vGrid[xId-floor(w/2.0)][y]->congestion();
                } else {
                    wCongestion += _obsCongest;
                }
            }
        } else if (dir == Direction::UpRight) {
            for (int x = xId-floor(w/2.0); x <= xId+floor(w/2.0); ++ x) {
                // cerr << "x = " << x << ", y = " << yId+floor(w/2.0) << endl;
                if (legal(x, yId+floor(w/2.0))) {
                    wCongestion += _vGrid[x][yId+floor(w/2.0)]->congestion();
                } else {
                    wCongestion += _obsCongest;
                }
            }
            for (int y = yId-floor(w/2.0); y <= yId+floor(w/2.0)-1; ++ y) {
                // cerr << "x = " << xId+floor(w/2.0) << ", y = " << y << endl;
                if (legal(xId+floor(w/2.0), y)) {
                    wCongestion += _vGrid[xId+floor(w/2.0)][y]->congestion();
                } else {
                    wCongestion += _obsCongest;
                }
            }
        } else if (dir == Direction::UpLeft) {
            for (int x = xId-floor(w/2.0); x <= xId+floor(w/2.0); ++ x) {
                // cerr << "x = " << x << ", y = " << yId+floor(w/2.0) << endl;
                if (legal(x, yId+floor(w/2.0))) {
                    wCongestion += _vGrid[x][yId+floor(w/2.0)]->congestion();
                } else {
                    wCongestion += _obsCongest;
                }
            }
            for (int y = yId-floor(w/2.0); y <= yId+floor(w/2.0)-1; ++ y) {
                // cerr << "x = " << xId-floor(w/2.0) << ", y = " << y << endl;
                if (legal(xId-floor(w/2.0), y)) {
                    wCongestion += _vGrid[xId-floor(w/2.0)][y]->congestion();
                } else {
                    wCongestion += _obsCongest;
                }
            }
        } else if (dir == Direction::DownRight) {
            for (int x = xId-floor(w/2.0); x <= xId+floor(w/2.0); ++ x) {
                // cerr << "x = " << x << ", y = " << yId-floor(w/2.0) << endl;
                if (legal(x, yId-floor(w/2.0))) {
                    wCongestion += _vGrid[x][yId-floor(w/2.0)]->congestion();
                } else {
                    wCongestion += _obsCongest;
                }
            }
            for (int y = yId-floor(w/2.0)+1; y <= yId+floor(w/2.0); ++ y) {
                // cerr << "x = " << xId+floor(w/2.0) << ", y = " << y << endl;
                if (legal(xId+floor(w/2.0), y)) {
                    wCongestion += _vGrid[xId+floor(w/2.0)][y]->congestion();
                } else {
                    wCongestion += _obsCongest;
                }
            }
        } else {
            assert(dir == Direction::DownLeft);
            for (int x = xId-floor(w/2.0); x <= xId+floor(w/2.0); ++ x) {
                // cerr << "x = " << x << ", y = " << yId-floor(w/2.0) << endl;
                if (legal(x, yId-floor(w/2.0))) {
                    wCongestion += _vGrid[x][yId-floor(w/2.0)]->congestion();
                } else {
                    wCongestion += _obsCongest;
                }
            }
            for (int y = yId-floor(w/2.0)+1; y <= yId+floor(w/2.0); ++ y) {
                // cerr << "x = " << xId-floor(w/2.0) << ", y = " << y << endl;
                if (legal(xId-floor(w/2.0), y)) {
                    wCongestion += _vGrid[xId-floor(w/2.0)][y]->congestion();
                } else {
                    wCongestion += _obsCongest;
                }
            }
        }
        wCongestion *= prob(w);
        congestion += wCongestion;
    }
    return congestion;
}

Direction AStarRouter::giveDirection (GNode* curNode){

    // Up, Down, Right, Left,
    // UpRight, UpLeft, DownRight, DownLeft
    assert( curNode->parent() != curNode);

    Direction dir;
    if (curNode->parent()->xId() == curNode->xId()+1 && curNode->parent()->yId() == curNode->yId()+1) {
        dir = Direction::UpRight;
    } 
    else if (curNode->parent()->xId() == curNode->xId()-1 && curNode->parent()->yId() == curNode->yId()+1 ) {
        dir = Direction::UpLeft;
    } 
    else if (curNode->parent()->xId() == curNode->xId()-1 && curNode->parent()->yId() == curNode->yId()-1) {
        dir = Direction::DownLeft;
    } 
    else if (curNode->parent()->xId() == curNode->xId()+1 && curNode->parent()->yId() == curNode->yId()-1){
        dir = Direction::DownRight;
    }
    else if (curNode->parent()->xId() == curNode->xId()+1) {
        dir = Direction::Right;
    } 
    else if (curNode->parent()->xId() == curNode->xId()-1) {
        dir = Direction::Left;
    } 
    else if (curNode->parent()->yId() == curNode->yId()+1) {
        dir = Direction::Up;
    } 
    else {
        assert(curNode->parent()->yId() == curNode->yId()-1);
        dir = Direction::Down;
    }
    return dir;
}

double AStarRouter::pathLength(int threshold, int method) {
    
    // 0 for heuristic
    if(method == 0){
        /////////////////////////
        //1st method: Heuristic//
        /////////////////////////

        //Situation 1: If walk diogonally(1 step), then move and pathLength += 2^(1/2)
        //Situation 2: If walk strqight and then walk diogonally(2 step), then move to 2nd step and pathLength += 1+2^(1/2)
        //Situation 3: If walk strqight and then turn right or left(2 step), then move to 2nd step and pathLength += 2^(1/2)
        //Situation 4: If walk strqight and then walk straight again(2 step), then move to 1st step and pathLength += 1

        int gridCount = 0;
        double pathLength = 0;
        int buffer = 0;
        bool turned = false;
        bool turn = false;
        GNode* node = _vGNode[_tPos.first][_tPos.second];
        //一開始的Node就是從Target開始找，所以直接用_tPos，也就是target Pos。
        // cout << "Coordinate of this pt -> X =" <<   _tPos.first << "Y = " << _tPos.second << endl;
        
        bool firstStepStraight = false;
        Direction dir, prevDir;

        while(node -> parent() != node) {
            dir = giveDirection(node);
            if( !firstStepStraight && (dir == UpRight || dir == UpLeft || dir == DownLeft || dir == DownRight) ){
                pathLength += pow(2, 0.5);
                node = node->parent();
                //不知道這樣合不合法
                ++ gridCount;
                ++ gridCount;
                prevDir = dir;
            }
            else if(!firstStepStraight && (dir == Up || dir == Right || dir == Down || dir == Left) ){
                firstStepStraight = true;
                node = node->parent();
                ++gridCount;
                prevDir = dir;
            }
            else if ( firstStepStraight && (dir == UpRight || dir == UpLeft || dir == DownLeft || dir == DownRight)) {
                firstStepStraight = false;
                pathLength += (1+pow(2, 0.5));
                node = node->parent();
                ++gridCount;
                ++gridCount;
                prevDir = dir;
            }
            else if (firstStepStraight && (prevDir != dir)){
                firstStepStraight = false;
                pathLength += (pow(2, 0.5));
                node = node->parent();
                ++gridCount;
                prevDir = dir;
            }
            else if (firstStepStraight && (prevDir == dir)){
                pathLength += 1;
                node = node->parent();
                ++gridCount;
                prevDir = dir;
            }

        }

        // buffer ++;
        //             if (!turned && buffer > threshold) {
        //                 pathLength += buffer;
        //                 buffer = 0;
        //             }
        //         } else {
        //             if (!turned) {
        //                 turned = true;
        //             }
        for (size_t pGridId = 0; pGridId < _path.size(); ++pGridId) {
            Grid* grid = _path[pGridId];
        }
        cout << endl;
        // cout << "Grid Count is  " << gridCount << endl;
        // cout << "Path Length is  " << pathLength << endl;

        // return gridCount;
        return pathLength;
    }
    // i for pattern matching 

    //這裡沒有return 東西
        
    assert(false);
    return 0.0;
}