using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;


public class UIBtn : MonoBehaviour
{
    public InputField TextInput;

    public void Test()
    {
        MainUpdate.Instance.ShowPathTest();
    }

    public void ShowPath()
    {
        var rInput = TextInput.text;

        var nNum = int.Parse(rInput);

        MainUpdate.Instance.ShowPath(nNum);
    }
}
