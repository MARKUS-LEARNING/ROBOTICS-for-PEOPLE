"""Site tooling: Quartz-compatible URLs and the Dataview compiler. Run: python3 -m unittest discover -s tools"""
import os, sys, unittest
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import vault, sitebuild as S


class Urls(unittest.TestCase):
    def test_slug_matches_quartz(self):
        self.assertEqual(S.slug('07_Robot_Operating_System_(ROS)/URDF.md'), '07_Robot_Operating_System_(ROS)/URDF')
        self.assertEqual(S.slug('a b/R&D 100%?.md'), 'a-b/R-and-D-100-percent')

    def test_url_is_encoded(self):
        self.assertTrue(S.url("x/Grübler's_Formula.md").endswith("/x/Gr%C3%BCbler%27s_Formula"))


class Dataview(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.q = S.Query(vault.Book(vault.Vault()))

    def test_backlink_list(self):
        out = self.q.run('LIST FROM "" WHERE contains(file.outlinks, [[Sensors]])')
        self.assertTrue(out.startswith('- [['))

    def test_table_with_headers_and_filter(self):
        out = self.q.run('TABLE level AS "Level", status\nFROM "04_Sensors_and_Perception"\nWHERE status = "keystone"\nSORT file.name ASC')
        self.assertIn('| Note | Level | Status |', out)
        self.assertEqual(out.count('keystone'), out.count('\n') - 1)

    def test_unsupported_stays_live(self):
        self.assertIsNone(self.q.run('TABLE length(rows) FROM "" GROUP BY level'))
        self.assertIsNone(self.q.run('LIST WHERE length(filter(file.outlinks, (l) => !l.file)) > 0'))

    def test_unwritten_notes_become_text(self):
        v = vault.Vault()
        n = v.notes[0]
        out, c = S.unlink_missing('See [[No_Such_Note_xyz|this]] and [[No_Such_Note_abc]] but `[[No_Such_Note_q]]`.', v, n)
        self.assertEqual((out, c), ('See this and No Such Note abc but `[[No_Such_Note_q]]`.', 2))


class NotePages(unittest.TestCase):
    def test_facet_tags(self):
        fm = {'level': 'advanced', 'status': 'draft', 'type': 'method', 'subfield': ['control', 'planning']}
        self.assertEqual(S.facet_tags(fm), ['level/advanced', 'status/draft', 'type/method', 'subfield/control', 'subfield/planning'])

    def test_meta_strip_has_read_first_next_up_and_graph_link(self):
        b = vault.Book(vault.Vault()); nxt = S.next_up(b)
        n = next(x for x in b.notes if x.name == 'Kalman_Filter')
        strip = S.meta_strip(n, b, nxt, 'https://example.org/site')
        self.assertIn('**Read first:**', strip)
        self.assertIn('**Next up:**', strip)
        self.assertIn('(https://example.org/site/explore/?focus=Kalman_Filter)', strip)
        self.assertIn('\n>\n', strip)          # each item on its own line inside the callout

    def test_light_default_patch_applies_cleanly(self):
        with open(os.path.join(vault.ROOT, 'site', 'patches', 'default-light.patch')) as fh:
            p = fh.read()
        self.assertIn('+const userPref = "light"', p)


if __name__ == '__main__':
    unittest.main()
