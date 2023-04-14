using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.IO;

namespace _026_005_Thinout
{
	class Program
	{
#if DEBUG
		static string BasePath = "../../../../pictures/026_004_reverse_renamecopy";
#else
		static string BasePath = "pictures/026_004_reverse_renamecopy";
#endif
		static void Main(string[] args)
		{
			/* カレントディレクトリとexeの場所を一致させる */
			Directory.SetCurrentDirectory(Path.GetDirectoryName(System.Reflection.Assembly.GetEntryAssembly().Location));

			/* ファイルリスト取得 */
			string[] files = Directory.GetFiles(BasePath, "*.jpg");

			/* ファイルリストを日付時刻順に突っ込む */
			IDictionary<string, List<string>> lines = new Dictionary<string, List<string>>();
			foreach (string filepath in files)
			{
				string filefullpath = new DirectoryInfo(filepath).FullName;
				string keystr = Path.GetFileName(filefullpath).Substring(0, 20);
				if (lines.ContainsKey(keystr))
				{
					lines[keystr].Add(filefullpath);
				}
				else
				{
					lines.Add(keystr, new List<string>() { filefullpath });
				}
			}

			/* バッチファイル生成 */
			string thinoutbat = new DirectoryInfo(BasePath).Parent.Parent.FullName + "/026_006_Thinout.bat";
			File.WriteAllText(thinoutbat, "rem 間引きのバッチ\r\n", System.Text.Encoding.GetEncoding("shift_jis"));
			File.AppendAllText(thinoutbat, "cd " + new DirectoryInfo(BasePath).Parent.FullName + "\r\n", System.Text.Encoding.GetEncoding("shift_jis"));
			File.AppendAllText(thinoutbat, "D:" + "\r\n", System.Text.Encoding.GetEncoding("shift_jis"));
			File.AppendAllText(thinoutbat, "rmdir /s /q 026_006_Thinout" + "\r\n", System.Text.Encoding.GetEncoding("shift_jis"));
			File.AppendAllText(thinoutbat, "mkdir 026_006_Thinout" + "\r\n", System.Text.Encoding.GetEncoding("shift_jis"));
			File.AppendAllText(thinoutbat, "cd 026_006_Thinout" + "\r\n", System.Text.Encoding.GetEncoding("shift_jis"));

			/* 間引きのcopyコマンド生成 */
			double step = (100.0/*(%)*/ + 33/*(%)*/) / lines.Count;	/* 本来は通り過ぎるまでの100%だけど、真ん中付近を重複させて133%までを取得する */
			double progres_s = 0.0;
			double progres_e = step;
			foreach (KeyValuePair<string, List<string>> line in lines)
			{
				int lpct = 0;
				foreach (string pictfile in line.Value)
				{
					double progresper = lpct / (double)line.Value.Count * 100;
					if (progres_s < 66)
					{
						if (progres_s <= progresper && progresper < progres_e)
							File.AppendAllText(thinoutbat, "copy " + pictfile + " " + Path.GetFileName(pictfile) + "\r\n", System.Text.Encoding.GetEncoding("shift_jis"));
						else if (progresper >= progres_e)
							break;
					}
					else {
						if ((progres_s-33) <= progresper && progresper < (progres_e-33))
							File.AppendAllText(thinoutbat, "copy " + pictfile + " " + Path.GetFileName(pictfile) + "\r\n", System.Text.Encoding.GetEncoding("shift_jis"));
						else if (progresper >= (progres_e-33))
							break;

					}

					lpct++;
				}
				progres_s = progres_e;
				progres_e += step;
			}

			return;
		}
	}
}
