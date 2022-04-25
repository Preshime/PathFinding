using System.Collections;
using System.Collections.Generic;
using System.Threading.Tasks;
using UnityEngine;

public class MainUpdate : MonoBehaviour
{
    public static MainUpdate Instance;

    private Ground[] mMap;

    public GameObject Arrow;
    public GameObject Point;
    public GameObject KeyPoint;
    public GameObject TargetPoint;
    public GameObject Way;

    public GameObject KeyPointParent;
    public GameObject WayParent;

    private void Awake()
    {
        MainUpdate.Instance = this;
    }

    const int WIDTH = 1000;

    Dictionary<Vector3, GameObject> Map = new Dictionary<Vector3, GameObject>();

    // Start is called before the first frame update
    void Start()
    {
        //初始化地图
        this.mMap = new Ground[WIDTH * WIDTH];

        for (int i = 0; i < WIDTH; i++)
        {
            for (int j = 0; j < WIDTH; j++)
            {
                int nIndex = i * WIDTH + j;
                this.mMap[nIndex] = new FourGround()
                {
                    Coordinate = new Vector2Int(i, j),
                    WorldPos = new Vector3(i + 0.5f, 0.001f, j + 0.5f),
                    NearBy = new List<Ground>(),
                    Cost = 1,
                    IsWall = Random.Range(0, 10) == 1 || (nIndex / WIDTH % 100 == 3 && nIndex % WIDTH % 100 != 49),
                };
            }
        }

        var rGo = new GameObject("Wall");

        for (int i = 0; i < this.mMap.Length; i++)
        {
            SetNearBy(i, i - WIDTH);
            SetNearBy(i, i + WIDTH);
            SetNearBy(i, i - 1);
            SetNearBy(i, i + 1);

            if (!this.mMap[i].CanUsed)
            {
                var rWall = GameObject.Instantiate(this.Point, rGo.transform);
                rWall.transform.position = this.mMap[i].WorldPos;
            }
        }

        void SetNearBy(int nOldIndex, int nNearIndex)
        {
            int nIndex = nNearIndex;
            if (nIndex >= 0 && nIndex < this.mMap.Length && (nOldIndex / WIDTH == nNearIndex / WIDTH || nOldIndex % WIDTH == nNearIndex % WIDTH))
            {
                if (this.mMap[nIndex] != null)
                {
                    this.mMap[nOldIndex].NearBy.Add(this.mMap[nIndex]);
                }
            }
        }

        //初始化寻路
        PathFinding.Instance = new PathFinding();
        PathFinding.Instance.Initialize(ref this.mMap);

        foreach (var rP in PathFinding.Instance.KeyPoint)
        {
            var rKeyPoint = GameObject.Instantiate(this.KeyPoint, this.KeyPointParent.transform);
            rKeyPoint.transform.position = rP.WorldPos;
        }
    }

    private void Update()
    {
        var r = PathFinding.Instance.rG;
        if (r != null)
        {
            int nLen = r.Count;
            for (int i = 1; i < nLen; i++)
            {
                Debug.DrawLine(r[i - 1].WorldPos + Vector3.up, r[i].WorldPos + Vector3.up, Color.red);
            }
        }
    }


    //按钮调用
    public void ShowPath(int nNum)
    {
        List<Ground> rList = new List<Ground>();
        for (int i = 0; i < 2 * nNum; i++)
        {
            var nIndex = Random.Range(0, this.mMap.Length);
            while (nIndex < this.mMap.Length)
            {
                var rGround = this.mMap[++nIndex % this.mMap.Length];
                if (rGround.CanUsed && !rList.Contains(rGround))
                {
                    rList.Add(rGround);
                    break;
                }
            }
        }
        var nLen = rList.Count / 2;
        var rListPath = new List<List<Ground>>();
        var rTick1 = System.Environment.TickCount;
        for (int i = 0; i < nLen; i++)
        {
            var rFrom = rList[2 * i];
            var rTarget = rList[2 * i + 1];

            //rFrom = this.mMap[0];
            //rTarget = this.mMap[999999];

            var rGo2 = GameObject.Instantiate(this.TargetPoint);
            rGo2.transform.position = rFrom.WorldPos;
            var rGo1 = GameObject.Instantiate(this.TargetPoint);
            rGo1.transform.position = rTarget.WorldPos;

            //PathFinding.Instance.GetPathWithOutKeyPoint(rFrom, rTarget, out bArrive,out var rPath);
            PathFinding.Instance.GetPathWithKeyPoint(rFrom, rTarget, out bool bArrive, out var rPath);
            
            if (bArrive)
            {
                rListPath.Add(rPath);
            }
        }
        var rTick2 = System.Environment.TickCount;
        Debug.LogError(rTick2 - rTick1);
        Debug.LogError($"共有{rListPath.Count}条到达路线");
        for (int i = 0; i < rListPath.Count; i++)
        {
            var rPath = rListPath[i];
            if (rPath.Count < this.mMap.Length)
            {
                var rParent = new GameObject(i.ToString());

                foreach (var rGround in rPath)
                {
                    var rGo = GameObject.Instantiate(this.Arrow, rParent.transform);
                    rGo.transform.position = rGround.WorldPos;
                }
            }
        }
    }

    //按钮调用
    public async Task ShowPathTest()
    {
        List<Ground> rList = new List<Ground>();
        for (int i = 0; i < 2; i++)
        {
            var nIndex = Random.Range(0, this.mMap.Length);
            while (nIndex < this.mMap.Length)
            {
                var rGround = this.mMap[++nIndex % this.mMap.Length];
                if (rGround.CanUsed && !rList.Contains(rGround))
                {
                    rList.Add(rGround);
                    break;
                }
            }
        }
        var nLen = rList.Count / 2;
        var rListPath = new List<List<Ground>>();
        var rFrom = rList[0];
        var rTarget = rList[1];

        //rFrom = this.mMap[0];
        //rTarget = this.mMap[999999];

        var rGo2 = GameObject.Instantiate(this.TargetPoint);
        rGo2.transform.position = rFrom.WorldPos;
        var rGo1 = GameObject.Instantiate(this.TargetPoint);
        rGo1.transform.position = rTarget.WorldPos;

        //PathFinding.Instance.GetPathWithOutKeyPoint(rFrom, rTarget, out bArrive,out var rPath);
        await PathFinding.Instance.GetPathWithKeyPointTest(rFrom, rTarget,
            (x) =>
            {
                if (Application.isPlaying)
                {
                    if (!this.Map.TryGetValue(x, out var rGo))
                    {
                        rGo = GameObject.Instantiate(this.Way, this.WayParent.transform);
                        rGo.transform.position = x;
                        this.Map.Add(x, rGo);
                    }
                    else
                    {

                    }
                }
            });

        Debug.LogError("演示结束，开始正式寻路");
        var rTick1 = System.Environment.TickCount;
        PathFinding.Instance.GetPathWithKeyPoint(rFrom, rTarget, out bool bArrive, out var rPath);
        var rTick2 = System.Environment.TickCount;

        Debug.LogError(rTick2 - rTick1);
        Debug.LogError($"共有{rListPath.Count}条到达路线");

        if (rPath.Count < this.mMap.Length)
        {
            var rParent = new GameObject(999.ToString());

            foreach (var rGround in rPath)
            {
                var rGo = GameObject.Instantiate(this.Arrow, rParent.transform);
                rGo.transform.position = rGround.WorldPos;
            }
        }

    }
}


public class FourGround : Ground
{
    public bool IsWall;
    public bool CanUsed => !this.IsWall;

    public Vector2Int Coordinate { get; set; }
    public Vector3 WorldPos { get; set; }

    public int Cost { get; set; }

    public List<Ground> NearBy { get; set; }

    public float Distance(Ground rTarget, bool bFastComp = false, bool rRealDistance = true)
    {
        var x = rTarget.Coordinate.x - this.Coordinate.x;
        var y = rTarget.Coordinate.y - this.Coordinate.y;
        if (bFastComp)
        {
            return Mathf.Abs(x) + Mathf.Abs(y);
        }
        else
        {
            var rPow = Mathf.Pow(x, 2) + Mathf.Pow(y, 2);
            return rRealDistance ? Mathf.Sqrt(rPow) : rPow;
        }
    }
}