using System;
using System.Collections;
using System.Collections.Generic;
using System.Threading;
using System.Threading.Tasks;
using UnityEngine;

public class PathFinding
{
    public static PathFinding Instance;

    private const int LEADNUMX = 100;
    private const int LEADNUMY = 100;
    private const int PATHMAXLEN = 1000000;
    private const int KEYPOINTSHARE = 50;


    private Ground[] mMap;

    private int mMinX;
    private int mMaxX;
    private int mMinY;
    private int mMaxY;

    private int mWidth;
    private int mLength;

    private int mMapMaxCost;

    public List<Ground> KeyPoint;

    public void Initialize(ref Ground[] rMap)
    {
        this.mMap = rMap;

        this.mMinX = int.MaxValue;
        this.mMaxX = int.MinValue;
        this.mMinY = int.MaxValue;
        this.mMaxY = int.MinValue;

        for (int i = 0, nLen = this.mMap.Length; i < nLen; i++)
        {
            var rGround = this.mMap[i];
            var rCoor = rGround.Coordinate;

            if (this.mMinX > rCoor.x)
            {
                this.mMinX = rCoor.x;
            }
            if (this.mMaxX < rCoor.x)
            {
                this.mMaxX = rCoor.x;
            }
            if (this.mMinY > rCoor.y)
            {
                this.mMinY = rCoor.y;
            }
            if (this.mMaxY < rCoor.y)
            {
                this.mMaxY = rCoor.y;
            }
            if (rGround.Cost > this.mMapMaxCost)
            {
                this.mMapMaxCost = rGround.Cost;
            }
        }

        this.mWidth = this.mMaxX - this.mMinX + 1;
        this.mLength = this.mMaxY - this.mMinY + 1;

        //关键点顺序扫描
        this.KeyPoint = new List<Ground>();
        int nKeyX = this.mMinX;
        int nKeyY = this.mMinY;
        int nKeyPointSpace = this.mWidth / KEYPOINTSHARE;
        if (nKeyPointSpace > 0)
        {
            while (nKeyY <= this.mMaxY)
            {
                while (nKeyX <= this.mMaxX)
                {
                    var nIndex = nKeyY * this.mWidth + nKeyX;
                    if (nIndex < rMap.Length)
                    {
                        var rPoint = rMap[nIndex];
                        if (rPoint.CanUsed)
                        {
                            this.KeyPoint.Add(rPoint);
                        }
                    }
                    nKeyX += nKeyPointSpace;
                }
                nKeyX = this.mMinX;
                nKeyY += nKeyPointSpace;
            }
        }
    }


    public List<Ground> rG;
    public async Task GetPathWithKeyPointTest(Ground rFrom, Ground rTarget, Action<Vector3> rAct = null)
    {
        var rPath = new List<Ground>();

        //通过方向找出关键点列表
        //方向
        var rUsedKeyPoint = this.GetUsedKeyPoint(rFrom, rTarget);

        var rDebugKeyPoint = new List<Ground>(rUsedKeyPoint);

        rG = rDebugKeyPoint;
        //异步画，逻辑走的不一样，但是内容相同
        await PathAsync(rUsedKeyPoint, rFrom, rAct);
    }


    public void GetPathWithKeyPoint(Ground rFrom, Ground rTarget, out bool bArrived, out List<Ground> rPath)
    {
        bArrived = false;
        rPath = new List<Ground>();

        //通过方向找出关键点列表

        var rUsedKeyPoint = this.GetUsedKeyPoint(rFrom, rTarget);

        var rFromPoint = rFrom;

        var rStep = new StepMap[this.mWidth, this.mLength];
        int nCurStep = 1;
        rStep[rFrom.Coordinate.x, rFrom.Coordinate.y] =
            new StepMap()
            {
                Step = nCurStep,
                From = rFrom
            };

        var nKeyIndex = 0;

        while (rUsedKeyPoint.Count > nKeyIndex)
        {
            nKeyIndex = this.SetPathMap(rFromPoint, rUsedKeyPoint, ref nCurStep, out bool bKeyPointArrive, ref rStep);
            rFromPoint = rUsedKeyPoint[nKeyIndex];
            for (int i = nKeyIndex; i >= 0; i--)
            {
                rUsedKeyPoint.RemoveAt(i);
            }
            bArrived = bKeyPointArrive;
            if (!bKeyPointArrive)
            {
                Debug.LogError("无可用路径");
                break;
            }
        }

        rPath = this.GetPathByStepMap(rFrom, rTarget, rStep);
    }

    public async Task PathAsync(List<Ground> rUsedKeyPoint, Ground rFromPoint, Action<Vector3> rAct)
    {
        var nKeyIndex = 0;
        var rStep = new StepMap[this.mWidth, this.mLength];
        int nCurStep = 1;

        rStep[rFromPoint.Coordinate.x, rFromPoint.Coordinate.y] = new StepMap()
        {
            Step = nCurStep,
            From = rFromPoint
        };

        while (rUsedKeyPoint.Count > nKeyIndex)
        {
            nKeyIndex = await PathTest_Async(rFromPoint, rUsedKeyPoint, nCurStep, rStep, rAct);

            rFromPoint = rUsedKeyPoint[nKeyIndex];
            for (int i = nKeyIndex; i >= 0; i--)
            {
                rUsedKeyPoint.RemoveAt(i);
            }
            if (nKeyIndex < 0)
            {
                Debug.LogError("无可用路径");
                break;
            }
        }
    }

    public async Task<int> PathTest_Async(Ground rFrom, List<Ground> rTargetList, int nStartStep, StepMap[,] rStep, Action<Vector3> rAct)
    {
        var bArrived = false;

        if (rTargetList == null || rTargetList.Count <= 0)
        {
            Debug.LogError("目标列表无内容");
            return -1;
        }

        var nCurTargetIndex = 0;
        var rTarget = rTargetList[nCurTargetIndex];

        //起点终点相同
        if (rTarget == rFrom)
        {
            bArrived = true;
            return nCurTargetIndex;
        }

        

        var rWaitDict = new Dictionary<int, List<Ground>>();
        var nCurStep = nStartStep;
        //初始化起点
        rWaitDict.Add(nCurStep, new List<Ground>());
        rWaitDict[nCurStep].Add(rFrom);


        int nSkipNum = 0;
        while (!bArrived)
        {
            rWaitDict.TryGetValue(nCurStep, out var rList);
            //var rList = rWaitDict[nCurStep];
            if (rList != null && rList.Count > 0)
            {
                nSkipNum = 0;
                for (int j = 0; j < rList.Count; j++)
                {
                    var rCurGround = rList[j];
                    int nBiggerX = rTarget.Coordinate.x.CompareTo(rCurGround.Coordinate.x);
                    int nBiggerY = rTarget.Coordinate.y.CompareTo(rCurGround.Coordinate.y);

                    var rNear = rCurGround.NearBy;// 若低优先级有更快的路且已经计算了stepmap，会在倒回计算路线的时候走最近的路
                                                  //真实步数存在step地图上
                    var nCurRealStep = rStep[rCurGround.Coordinate.x, rCurGround.Coordinate.y].Step;
                    for (int i = 0; i < rNear.Count; i++)
                    {
                        var rNext = rNear[i];

                        if (!rNext.CanUsed)
                        {
                            rStep[rNext.Coordinate.x, rNext.Coordinate.y].Step = -1;
                        }
                        else
                        {
                            //对临时的计算地图对步数赋值
                            int nComputeStep = nCurStep + 1;
                            var rOldStep = rStep[rNext.Coordinate.x, rNext.Coordinate.y];
                            int nRealStep = nCurRealStep + rNext.Cost;

                            var nNextIndex = rTargetList.IndexOf(rNext);
                            if (nNextIndex >= nCurTargetIndex)
                            {
                                if (rOldStep.Step > nRealStep || rOldStep.Step == 0)
                                {
                                    rStep[rNext.Coordinate.x, rNext.Coordinate.y].Step = nRealStep;
                                }
                                rStep[rNext.Coordinate.x, rNext.Coordinate.y].From = rFrom;
                                nCurTargetIndex = nNextIndex;
                                bArrived = true;

                                return nCurTargetIndex;
                            }

                            if (rOldStep.Step == 0 || rOldStep.From  != rFrom || rOldStep.Step > nRealStep)
                            {
                                if (rOldStep.Step > nRealStep || rOldStep.Step == 0)
                                {
                                    rStep[rNext.Coordinate.x, rNext.Coordinate.y].Step = nRealStep;
                                }
                                rStep[rNext.Coordinate.x, rNext.Coordinate.y].From = rFrom;

                                rAct?.Invoke(rNext.WorldPos);

                                await Task.Delay(1);

                                //计算出当前步数后根据优先级计算出下次需要计算时的步数
                                if (rNext.Coordinate.x.CompareTo(rCurGround.Coordinate.x) != nBiggerX)
                                {
                                    nComputeStep += Mathf.Abs(rNext.Coordinate.x.CompareTo(rCurGround.Coordinate.x) - nBiggerX) * LEADNUMX;
                                }
                                if (rNext.Coordinate.y.CompareTo(rCurGround.Coordinate.y) != nBiggerY)
                                {
                                    nComputeStep += Mathf.Abs(rNext.Coordinate.y.CompareTo(rCurGround.Coordinate.y) - nBiggerY) * LEADNUMY;
                                }
                                if (!rWaitDict.ContainsKey(nComputeStep))
                                {
                                    rWaitDict.Add(nComputeStep, new List<Ground>());
                                }
                                rWaitDict[nComputeStep].Add(rNext);
                            }
                        }
                    }
                }
            }
            else
            {
                //已经没有需要计算的内容了
                if (nSkipNum > LEADNUMX + LEADNUMY + this.mMapMaxCost)
                {
                    break;
                }
                nSkipNum++;
            }
            nCurStep++;
        }
        return bArrived ? nCurTargetIndex : -1;
    }


    /// <summary>
    /// 获取关键点路径，此路径无视障碍物，只看关键点列表
    /// </summary>
    /// <param name="rFrom"></param>
    /// <param name="rTarget"></param>
    /// <returns></returns>
    private List<Ground> GetUsedKeyPoint(Ground rFrom, Ground rTarget)
    {
        Ground rCurFrom = rFrom;
        Ground rCurTarget = rTarget;

        List<Ground> rAlternate = new List<Ground>(this.KeyPoint);

        var rUsedKeyPoint = new List<Ground>();
        rUsedKeyPoint.Add(rFrom);

        do
        {
            int nMinX = Math.Min(rCurFrom.Coordinate.x, rCurTarget.Coordinate.x);
            int nMaxX = Math.Max(rCurFrom.Coordinate.x, rCurTarget.Coordinate.x);
            int nMinY = Math.Min(rCurFrom.Coordinate.y, rCurTarget.Coordinate.y);
            int nMaxY = Math.Max(rCurFrom.Coordinate.y, rCurTarget.Coordinate.y);

            rAlternate.RemoveAll((rGround) => rGround == rCurFrom || rGround.Coordinate.x < nMinX || rGround.Coordinate.x > nMaxX || rGround.Coordinate.y < nMinY || rGround.Coordinate.y > nMaxY);

            Ground rTemp = null;
            var fLastDis = 0f;
            var fCompLastDis = 0f;
            foreach (var rGround in rAlternate)
            {
                //无内容/未赋值/快速计算距离符合内容，若快速计算相等，则计算实际距离（未开根号）
                var rCurDis = rCurFrom.Distance(rGround, true);
                if (rTemp == null || fLastDis == 0 || fLastDis > rCurDis || (fLastDis == rCurDis &&  fCompLastDis > rGround.Distance(rTarget, false, false)))
                {
                    rTemp = rGround;

                    fLastDis = rCurFrom.Distance(rTemp, true);
                    fCompLastDis = rTemp.Distance(rTarget, false, false);
                }
            }

            if (rTemp != null)
            {
                rCurFrom = rTemp;
            }

            if (!rUsedKeyPoint.Contains(rCurFrom))
            {
                rUsedKeyPoint.Add(rCurFrom);
            }

        }
        while (rAlternate.Count > 1);
        rUsedKeyPoint.Add(rTarget);

        return rUsedKeyPoint;
    }

    public void GetPathWithOutKeyPoint(Ground rFrom, Ground rTarget, out bool bArrived, out List<Ground> rPath)
    {
        rPath = new List<Ground>();
        bArrived = false;

        if (rFrom.Coordinate.x < this.mMinX ||
           rFrom.Coordinate.x > this.mMaxX ||
           rFrom.Coordinate.y < this.mMinY ||
           rFrom.Coordinate.y > this.mMaxY ||
           rTarget.Coordinate.x < this.mMinX ||
           rTarget.Coordinate.x > this.mMaxX ||
           rTarget.Coordinate.y < this.mMinY ||
           rTarget.Coordinate.y > this.mMaxY)
        {
            throw new Exception("起始点或目标点不在地图范围内");
        }

        //初始化寻路map，数组中默认值为0，所以把step+1，从1开始
        var rStep = new StepMap[this.mWidth, this.mLength];
        int nCurStep = 1;
        rStep[rFrom.Coordinate.x, rFrom.Coordinate.y] = new StepMap() { Step = nCurStep, From = rFrom };



        this.SetPathMap(rFrom, new List<Ground>(new Ground[] { rTarget }), ref nCurStep, out bArrived, ref rStep);

        //从目标点倒序找到路线，如果没到就从周围找，直到某一点的相邻点可以到达且取其中步数最少的点
        rPath = this.GetPathByStepMap(rFrom, rTarget, rStep);
    }

    private int SetPathMap(Ground rFrom, List<Ground> rTargetList, ref int nStartStep, out bool bArrived, ref StepMap[,] rStep)
    {
        bArrived = false;

        if (rTargetList == null || rTargetList.Count <= 0)
        {
            Debug.LogError("目标列表无内容");
            return -1;
        }

        var nCurTargetIndex = 0;
        var rTarget = rTargetList[nCurTargetIndex];

        //起点终点相同
        if (rTarget == rFrom)
        {
            bArrived = true;
            return nCurTargetIndex;
        }

        var rWaitDict = new Dictionary<int, List<Ground>>();
        var nCurStep = nStartStep;
        //初始化起点
        rWaitDict.Add(nCurStep, new List<Ground>());
        rWaitDict[nCurStep].Add(rFrom);


        int nSkipNum = 0;
        while (!bArrived)
        {
            rWaitDict.TryGetValue(nCurStep, out var rList);
            //var rList = rWaitDict[nCurStep];
            if (rList != null && rList.Count > 0)
            {
                nSkipNum = 0;
                for (int j = 0; j < rList.Count; j++)
                {
                    var rCurGround = rList[j];

                    int nBiggerX = rTarget.Coordinate.x.CompareTo(rCurGround.Coordinate.x);
                    int nBiggerY = rTarget.Coordinate.y.CompareTo(rCurGround.Coordinate.y);

                    var rNear = rCurGround.NearBy;// 若低优先级有更快的路且已经计算了stepmap，会在倒回计算路线的时候走最近的路
                    //真实步数存在step地图上
                    var nCurRealStep = rStep[rCurGround.Coordinate.x, rCurGround.Coordinate.y].Step;
                    for (int i = 0; i < rNear.Count; i++)
                    {
                        var rNext = rNear[i];

                        if (!rNext.CanUsed)
                        {
                            rStep[rNext.Coordinate.x, rNext.Coordinate.y].Step = -1;
                            //var nNextIndex = rTargetList.IndexOf(rNext);
                            //if (nNextIndex >= nCurTargetIndex)
                            //{
                            //    nCurTargetIndex = nNextIndex;
                            //    rTarget = rTargetList[nCurTargetIndex];

                            //    continue;
                            //}
                        }
                        else
                        {
                            //对临时的计算地图对步数赋值
                            int nComputeStep = nCurStep + 1;
                            StepMap rOldStep = rStep[rNext.Coordinate.x, rNext.Coordinate.y];
                            int nRealStep = nCurRealStep + rNext.Cost;

                            var nNextIndex = rTargetList.IndexOf(rNext);
                            if (nNextIndex >= nCurTargetIndex)
                            {
                                if (rOldStep.Step > nRealStep || rOldStep.Step == 0)
                                {
                                    rStep[rNext.Coordinate.x, rNext.Coordinate.y].Step = nRealStep;
                                }
                                rStep[rNext.Coordinate.x, rNext.Coordinate.y].From = rFrom;
                                nCurTargetIndex = nNextIndex;
                                bArrived = true;

                                return nCurTargetIndex;
                            }

                            if (rOldStep.From != rFrom || rOldStep.Step > nRealStep || rOldStep.Step == 0)
                            {
                                if (rOldStep.Step > nRealStep || rOldStep.Step == 0)
                                {
                                    rStep[rNext.Coordinate.x, rNext.Coordinate.y].Step = nRealStep; 
                                }
                                rStep[rNext.Coordinate.x, rNext.Coordinate.y].From = rFrom;

                                //计算出当前步数后根据优先级计算出下次需要计算时的步数
                                if (rNext.Coordinate.x.CompareTo(rCurGround.Coordinate.x) != nBiggerX)
                                {
                                    nComputeStep += Mathf.Abs(rNext.Coordinate.x.CompareTo(rCurGround.Coordinate.x) - nBiggerX) * LEADNUMX;
                                }
                                if (rNext.Coordinate.y.CompareTo(rCurGround.Coordinate.y) != nBiggerY)
                                {
                                    nComputeStep += Mathf.Abs(rNext.Coordinate.y.CompareTo(rCurGround.Coordinate.y) - nBiggerY) * LEADNUMY;
                                }
                                if (!rWaitDict.ContainsKey(nComputeStep))
                                {
                                    rWaitDict.Add(nComputeStep, new List<Ground>());
                                }
                                rWaitDict[nComputeStep].Add(rNext);
                            }
                        }
                    }
                }
            }
            else
            {
                //已经没有需要计算的内容了
                if (nSkipNum > LEADNUMX + LEADNUMY + this.mMapMaxCost)
                {
                    break;
                }
                nSkipNum++;
            }
            nCurStep++;
        }
        return nCurTargetIndex;
    }

    /// <summary>
    /// 从目标点倒序找到路线，如果没到就从周围找，直到某一点的相邻点可以到达且取其中步数最少的点
    /// </summary>
    /// <param name="rFrom"></param>
    /// <param name="rTarget"></param>
    /// <param name="rStep"></param>
    /// <returns></returns>
    private List<Ground> GetPathByStepMap(Ground rFrom, Ground rTarget, StepMap[,] rStep)
    {
        var rPath = new List<Ground>();
        var nCurStep = 0;
        Ground rGround = rTarget;
        try
        {
            int ii = 0;
            rPath.Add(rFrom);
            while (ii++ < PATHMAXLEN)
            {
                rPath.Add(rGround);
                nCurStep = rStep[rGround.Coordinate.x, rGround.Coordinate.y].Step;
                var nLastStep = nCurStep - rGround.Cost;
                var rNear = rGround.NearBy;
                for (int i = 0; i < rNear.Count; i++)
                {
                    var rCoor = rNear[i].Coordinate;
                    var nNearStep = rStep[rCoor.x, rCoor.y].Step;
                    if (nLastStep >= nNearStep && nNearStep > 0)
                    {
                        rGround = rNear[i];
                        break;
                    }
                }
                if (rGround == rFrom)
                {
                    break;
                }
            }
        }
        catch (Exception e)
        {
            Debug.LogError(e.Message);
        }
        rPath.Reverse();
        return rPath;
    }
}

public interface Ground
{
    Vector2Int Coordinate { get; set; }
    Vector3 WorldPos { get; set; }
    bool CanUsed { get; }

    List<Ground> NearBy { get; set; }

    int Cost { get; set; }

    float Distance(Ground rTarget, bool bFastComp = false, bool bRealDistance = true);
}

public struct StepMap
{
    public int Step;
    public Ground From;
}